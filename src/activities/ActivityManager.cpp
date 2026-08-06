#include "ActivityManager.h"

#include <FontCacheManager.h>
#include <HalPowerManager.h>

#include <algorithm>

#include "OpdsServerStore.h"
#include "boot_sleep/BootActivity.h"
#include "boot_sleep/SleepActivity.h"
#include "browser/OpdsBookBrowserActivity.h"
#include "home/CrashActivity.h"
#include "home/FileBrowserActivity.h"
#include "home/HomeActivity.h"
#include "home/RecentBooksActivity.h"
#include "network/CrossPointWebServerActivity.h"
#include "reader/ReaderActivity.h"
#include "settings/OpdsServerListActivity.h"
#include "settings/SettingsActivity.h"
#include "util/FullScreenMessageActivity.h"

static portMUX_TYPE activityManagerSpinlock = portMUX_INITIALIZER_UNLOCKED;
namespace {
// The foreground B/W waveform itself is the cancellation window: any input
// already queued while it runs invalidates the generation before maintenance.
// Do not add a visible post-refresh delay before gray refinement or cleanup.
constexpr uint32_t DISPLAY_MAINTENANCE_QUIET_MS = 0;

// How long the controller stays powered after the queue drains. Deliberately
// not fused with the quiet window above: that one gates waveforms the user can
// see and must stay at zero, this one gates pure housekeeping and wants to
// outlast the gap between two page turns.
constexpr uint32_t CONTROLLER_POWER_OFF_IDLE_MS = 1500;
}

void ActivityManager::begin() {
#if defined(configNUM_CORES) && configNUM_CORES > 1
  constexpr BaseType_t renderTaskCore = 1;
#else
  constexpr BaseType_t renderTaskCore = 0;
#endif
  xTaskCreatePinnedToCore(&renderTaskTrampoline, "ActivityManagerRender",
                          8192,               // Stack size
                          this,               // Parameters
                          1,                  // Priority
                          &renderTaskHandle,  // Task handle
                          renderTaskCore  // Keep long renders/cover decodes off CPU 0's idle watchdog when available
  );
  assert(renderTaskHandle != nullptr && "Failed to create render task");
}

void ActivityManager::renderTaskTrampoline(void* param) {
  auto* self = static_cast<ActivityManager*>(param);
  self->renderTaskLoop();
}

void ActivityManager::renderTaskLoop() {
  uint32_t renderedSequence = 0;
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (true) {
      const uint32_t sequence = updateSequence.load();
      // Sampled together with the queue so an input edge arriving while we wait
      // for RenderLock is not silently absorbed below. requestUpdate() aborts
      // before it bumps updateSequence, so an abort paired with a new frame is
      // already covered by `sequence`; noteUserInteraction() deliberately does
      // not bump updateSequence, and that is the case this closes.
      const uint32_t interactionAtQueue = interactionSequence.load();
      unsigned long renderStarted = 0;
      unsigned long foregroundDone = 0;
      if (sequence != renderedSequence) {
        renderStarted = millis();
        const unsigned long requestedAt = lastUpdateRequestedMs.load();
        LOG_DBG("ACT", "Render start: seq=%lu queued=%lums", static_cast<unsigned long>(sequence),
                requestedAt == 0 ? 0 : renderStarted - requestedAt);
        // Acquire the lock before reading currentActivity to avoid a TOCTOU race
        // where the main task deletes the activity between the null-check and render().
        RenderLock lock;
        // The wait above can be seconds (a screenshot runs two full refreshes).
        // beginDisplayWork() snapshots _abortGeneration as its new baseline, so
        // an abort raised during that wait would be erased and this render would
        // run to completion -- a full 3-gray composition plus its activation --
        // for input the user has already superseded. Drop back instead;
        // renderedSequence is untouched and interactionAtQueue is re-sampled on
        // the next pass, so the frame is retried, not lost.
        if (interactionSequence.load() != interactionAtQueue) {
          continue;  // RenderLock releases via RAII
        }
        if (currentActivity) {
          HalPowerManager::Lock powerLock;
          // Bind cancellation before any CPU-side composition. An input edge
          // which lands during layout then cancels this render's gray/cleanup
          // tail instead of being erased when displayStart() is reached.
          renderer.beginDisplayWork();
          currentActivity->render(std::move(lock));
        }
        lock.unlock();
        foregroundDone = millis();
        renderedSequence = sequence;

        // Wake a synchronous caller only after the generation it requested has
        // actually rendered, not after an older in-flight render happens to end.
        TaskHandle_t waiter = nullptr;
        taskENTER_CRITICAL(&activityManagerSpinlock);
        if (waitingTaskHandle && sequence >= waitingUpdateSequence) {
          waiter = waitingTaskHandle;
          waitingTaskHandle = nullptr;
          waitingUpdateSequence = 0;
        }
        taskEXIT_CRITICAL(&activityManagerSpinlock);
        if (waiter) xTaskNotify(waiter, 1, eIncrement);
      }

      // render() may request a follow-up frame (home cover completion, async
      // data becoming ready, etc.) without notifying this task until the main
      // loop drains requestedUpdate. updateSequence is already authoritative,
      // so consume that foreground frame now. Treating this window as idle
      // inserted a deghost waveform between two halves of one UI transition.
      if (updateSequence.load() != renderedSequence) {
        continue;
      }

      const uint32_t quietInteraction = interactionSequence.load();
      const uint32_t quietUpdate = updateSequence.load();
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DISPLAY_MAINTENANCE_QUIET_MS)) > 0) {
        continue;
      }
      if (interactionSequence.load() != quietInteraction || updateSequence.load() != quietUpdate) {
        continue;
      }

      // Touch taps are classified on release by the main task. A press may
      // arrive while a waveform is BUSY, and after BUSY clears there is a
      // narrow handoff before the release becomes a page-turn request. Do not
      // start deghost or spend 140 ms powering the controller off in that
      // window. finishUserInteractionDispatch() wakes us after the Activity
      // has either queued the next frame or consumed the gesture.
      if (userInputActive.load() || interactionDispatchPending.load()) {
        break;
      }

      if (!renderer.hasPendingDisplayMaintenance()) {
        // Recheck both generations before declaring the queue idle, then let
        // Paper Mono shut down analog/clock domains retained across adjacent
        // waveforms.
        if (interactionSequence.load() != quietInteraction || updateSequence.load() != quietUpdate) {
          continue;
        }
        if (renderStarted != 0) {
          LOG_DBG("ACT", "Render complete: seq=%lu foreground=%lums maintenance=0ms",
                  static_cast<unsigned long>(sequence), foregroundDone - renderStarted);
        }

        // Unlike the maintenance waveforms above, nothing on screen depends on
        // the power-off, so it waits for the reader to actually stop turning
        // pages. It costs ~140 ms of BUSY under RenderLock, so a turn landing
        // inside it blocks on the lock and then pays initController() to undo
        // the shutdown -- ~180 ms added to precisely the turn the user
        // experiences as the quick one. Both requestUpdate() and
        // noteUserInteraction() notify this task, so the window collapses the
        // instant there is work; the only cost to a device the user has put
        // down is the controller's retained analog domains for another second
        // and a half.
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONTROLLER_POWER_OFF_IDLE_MS)) > 0) {
          continue;
        }
        // hasPendingMaintenance() is re-read too: the main task also drives the
        // panel under RenderLock (forced refresh, screenshots) and can queue a
        // gray refinement during the window without bumping either sequence.
        if (interactionSequence.load() != quietInteraction || updateSequence.load() != quietUpdate ||
            userInputActive.load() || interactionDispatchPending.load() ||
            renderer.hasPendingDisplayMaintenance()) {
          continue;
        }
        {
          // This task is NOT the sole controller consumer: the main task drives
          // the panel under RenderLock for forced refresh (main.cpp) and
          // screenshots. controllerIdle() spends ~140 ms powering off and then
          // deep-sleeps the controller, so running it unlocked let the main task
          // enter writePlane() concurrently -- two owners of the file-static
          // BUSY semaphore and its shared CHANGE interrupt (EpdBus.cpp), of the
          // single 16 KB ROTATE_CHUNK staging buffer, and of the SPI
          // transaction. Take the same lock the foreground path uses. The
          // earlier RenderLock is already released above, and RenderLock is
          // non-recursive, so this cannot self-deadlock.
          RenderLock idleLock;
          HalPowerManager::Lock powerLock;
          renderer.displayControllerIdle();
        }
        break;
      }

      // Foreground updates always win above; while the controller is otherwise
      // idle, drain exactly one low-priority maintenance waveform and then
      // re-check both queues. RenderLock is required for the same reason as the
      // idle power-off: the main task is a second controller consumer.
      const unsigned long maintenanceStarted = millis();
      displayControllerWorkActive.store(true);
      {
        RenderLock maintenanceLock;
        HalPowerManager::Lock powerLock;
        renderer.runDisplayMaintenance();
      }
      displayControllerWorkActive.store(false);
      const unsigned long maintenanceMs = millis() - maintenanceStarted;
      LOG_DBG("ACT", "Controller maintenance task: %lums pending=%u", maintenanceMs,
              static_cast<unsigned>(renderer.hasPendingDisplayMaintenance()));

      // Loop even when this was the final task: a touch/update may have raced
      // its indivisible waveform, and the foreground queue must be checked
      // before this controller worker sleeps again.
      continue;
    }
  }
}

void ActivityManager::loop() {
  if (currentActivity) {
    if (!currentActivity->isHomeActivity() && mappedInput.wasHomeGesture()) {
      if (currentActivity->handleHomeGesture()) {
        return;
      }
      goHome();
      return;
    }

    // Note: do not hold a lock here, the loop() method must be responsible for acquire one if needed
    currentActivity->loop();
  }

  while (pendingAction != PendingAction::None) {
    if (pendingAction == PendingAction::Pop) {
      RenderLock lock;

      if (!currentActivity) {
        // Should never happen in practice
        LOG_ERR("ACT", "Pop set but currentActivity is null; ignoring pop request");
        pendingAction = PendingAction::None;
        continue;
      }

      ActivityResult pendingResult = std::move(currentActivity->result);

      // Destroy the current activity
      exitActivity(lock);
      pendingAction = PendingAction::None;

      if (stackActivities.empty()) {
        LOG_DBG("ACT", "No more activities on stack, going home");
        lock.unlock();  // goHome may acquire its own lock
        goHome();
        continue;  // Will launch goHome immediately

      } else {
        currentActivity = std::move(stackActivities.back());
        stackActivities.pop_back();
        LOG_DBG("ACT", "Popped from activity stack, new size = %zu", stackActivities.size());
        // Handle result if necessary
        if (currentActivity->resultHandler) {
          LOG_DBG("ACT", "Handling result for popped activity");

          // Move it here to avoid the case where handler calling another startActivityForResult()
          auto handler = std::move(currentActivity->resultHandler);
          currentActivity->resultHandler = nullptr;
          lock.unlock();  // Handler may acquire its own lock
          handler(pendingResult);
        }

        // Request an update to ensure the popped activity gets re-rendered
        if (pendingAction == PendingAction::None) {
          requestUpdate();
        }

        // Handler may request another pending action, we will handle it in the next loop iteration
        continue;
      }

    } else if (pendingActivity) {
      // Current activity has requested a new activity to be launched
      RenderLock lock;

      if (pendingAction == PendingAction::Replace) {
        // Destroy the current activity
        exitActivity(lock);
        // Clear the stack
        while (!stackActivities.empty()) {
          stackActivities.back()->onExit();
          stackActivities.pop_back();
        }
      } else if (pendingAction == PendingAction::Push) {
        // Move current activity to stack
        stackActivities.push_back(std::move(currentActivity));
        LOG_DBG("ACT", "Pushed to activity stack, new size = %zu", stackActivities.size());
      }
      pendingAction = PendingAction::None;
      currentActivity = std::move(pendingActivity);

      lock.unlock();  // onEnter may acquire its own lock
      currentActivity->onEnter();

      // onEnter may request another pending action, we will handle it in the next loop iteration
      continue;
    }
  }

  if (requestedUpdate.exchange(false)) {
    // Using direct notification to signal the render task to update
    // Increment counter so multiple rapid calls won't be lost
    if (renderTaskHandle) {
      xTaskNotify(renderTaskHandle, 1, eIncrement);
    }
  }
}

void ActivityManager::exitActivity(const RenderLock& lock) {
  // Note: lock must be held by the caller
  if (currentActivity) {
    currentActivity->onExit();
    currentActivity.reset();
  }
}

void ActivityManager::replaceActivity(std::unique_ptr<Activity>&& newActivity) {
  // Note: no lock here, this is usually called by loop() and we may run into deadlock
  if (currentActivity) {
    // Defer launch if we're currently in an activity, to avoid deleting the current activity
    // leading to the "delete this" problem
    pendingActivity = std::move(newActivity);
    pendingAction = PendingAction::Replace;
  } else {
    // No current activity, safe to launch immediately
    currentActivity = std::move(newActivity);
    currentActivity->onEnter();
  }
}

void ActivityManager::goToFileTransfer() {
  replaceActivity(std::make_unique<CrossPointWebServerActivity>(renderer, mappedInput));
}

void ActivityManager::goToSettings() { replaceActivity(std::make_unique<SettingsActivity>(renderer, mappedInput)); }

void ActivityManager::goToFileBrowser(std::string path) {
  replaceActivity(std::make_unique<FileBrowserActivity>(renderer, mappedInput, std::move(path)));
}

void ActivityManager::goToRecentBooks() {
  replaceActivity(std::make_unique<RecentBooksActivity>(renderer, mappedInput));
}

void ActivityManager::goToBrowser() {
  const auto& servers = OPDS_STORE.getServers();
  // Skip the server picker when there's only one server configured
  if (servers.size() == 1) {
    replaceActivity(std::make_unique<OpdsBookBrowserActivity>(renderer, mappedInput, servers[0]));
  } else {
    replaceActivity(std::make_unique<OpdsServerListActivity>(renderer, mappedInput, true));
  }
}

void ActivityManager::goToReader(std::string path, const bool allowFastInitialRefresh) {
  replaceActivity(std::make_unique<ReaderActivity>(renderer, mappedInput, std::move(path), allowFastInitialRefresh));
}

void ActivityManager::goToSleep(bool fromTimeout) {
  replaceActivity(std::make_unique<SleepActivity>(renderer, mappedInput, fromTimeout));
  loop();  // Important: sleep screen must be rendered immediately, the caller will go to sleep right after this returns
}

void ActivityManager::goToBoot() { replaceActivity(std::make_unique<BootActivity>(renderer, mappedInput)); }

void ActivityManager::goToFullScreenMessage(std::string message, EpdFontFamily::Style style) {
  replaceActivity(std::make_unique<FullScreenMessageActivity>(renderer, mappedInput, std::move(message), style));
}

void ActivityManager::goHome(HomeMenuItem initialMenuItem) {
  const bool refreshAfterEnter = isReaderActivity();
  if (initialMenuItem == HomeMenuItem::NONE && currentActivity) {
    const auto& activityName = currentActivity->name;
    if (activityName == "FileBrowser") {
      initialMenuItem = HomeMenuItem::FILE_BROWSER;
    } else if (activityName == "RecentBooks") {
      initialMenuItem = HomeMenuItem::RECENTS;
    } else if (activityName == "OpdsBookBrowser") {
      initialMenuItem = HomeMenuItem::OPDS_BROWSER;
    } else if (activityName == "CrossPointWebServer") {
      initialMenuItem = HomeMenuItem::FILE_TRANSFER;
    } else if (activityName == "Settings") {
      initialMenuItem = HomeMenuItem::SETTINGS_MENU;
    }
  }
  replaceActivity(std::make_unique<HomeActivity>(renderer, mappedInput, initialMenuItem, refreshAfterEnter));
}
void ActivityManager::goToCrashReport() { replaceActivity(std::make_unique<CrashActivity>(renderer, mappedInput)); }

void ActivityManager::pushActivity(std::unique_ptr<Activity>&& activity) {
  if (pendingActivity) {
    // Should never happen in practice
    LOG_ERR("ACT", "pendingActivity while pushActivity is not expected");
    pendingActivity.reset();
  }
  pendingActivity = std::move(activity);
  pendingAction = PendingAction::Push;
}

void ActivityManager::popActivity() {
  if (pendingActivity) {
    // Should never happen in practice
    LOG_ERR("ACT", "pendingActivity while popActivity is not expected");
    pendingActivity.reset();
  }
  pendingAction = PendingAction::Pop;
}

bool ActivityManager::preventAutoSleep() const { return currentActivity && currentActivity->preventAutoSleep(); }

bool ActivityManager::isReaderActivity() const {
  return std::any_of(stackActivities.begin(), stackActivities.end(),
                     [](const auto& activity) { return activity->isReaderActivity(); }) ||
         (currentActivity && currentActivity->isReaderActivity());
}

bool ActivityManager::handleForcedRefresh() { return currentActivity && currentActivity->handleForcedRefresh(); }

bool ActivityManager::skipLoopDelay() const { return currentActivity && currentActivity->skipLoopDelay(); }

ScreenshotInfo ActivityManager::getScreenshotInfo() const {
  if (currentActivity) {
    return currentActivity->getScreenshotInfo();
  }
  return {};
}

void ActivityManager::requestUpdate(bool immediate) {
  renderer.abortDisplayWork();
  lastUpdateRequestedMs.store(millis());
  updateSequence.fetch_add(1);
  if (immediate) {
    if (renderTaskHandle) {
      xTaskNotify(renderTaskHandle, 1, eIncrement);
    }
  } else {
    // Deferring the update until current loop is finished
    // This is to avoid multiple updates being requested in the same loop
    requestedUpdate = true;
  }
}

void ActivityManager::noteUserInteraction() {
  interactionDispatchPending.store(true);
  renderer.abortDisplayWork();
  interactionSequence.fetch_add(1);
  if (renderTaskHandle) xTaskNotify(renderTaskHandle, 1, eIncrement);
}

void ActivityManager::setUserInputActive(const bool active) {
  const bool previous = userInputActive.exchange(active);
  if (previous != active && renderTaskHandle) xTaskNotify(renderTaskHandle, 1, eIncrement);
}

void ActivityManager::finishUserInteractionDispatch() {
  if (interactionDispatchPending.exchange(false) && renderTaskHandle) {
    xTaskNotify(renderTaskHandle, 1, eIncrement);
  }
}

void ActivityManager::requestUpdateAndWait() {
  if (!renderTaskHandle) {
    return;
  }
  renderer.abortDisplayWork();
  lastUpdateRequestedMs.store(millis());
  const uint32_t sequence = updateSequence.fetch_add(1) + 1;

  // Atomic section to perform checks
  taskENTER_CRITICAL(&activityManagerSpinlock);
  auto currTaskHandler = xTaskGetCurrentTaskHandle();
  auto mutexHolder = xSemaphoreGetMutexHolder(renderingMutex);
  bool isRenderTask = (currTaskHandler == renderTaskHandle);
  bool alreadyWaiting = (waitingTaskHandle != nullptr);
  bool holdingRenderLock = (mutexHolder == currTaskHandler);
  if (!alreadyWaiting && !isRenderTask && !holdingRenderLock) {
    waitingTaskHandle = currTaskHandler;
    waitingUpdateSequence = sequence;
  }
  taskEXIT_CRITICAL(&activityManagerSpinlock);

  // Render task cannot call requestUpdateAndWait() or it will cause a deadlock
  assert(!isRenderTask && "Render task cannot call requestUpdateAndWait()");

  // There should never be the case where 2 tasks are waiting for a render at the same time
  assert(!alreadyWaiting && "Already waiting for a render to complete");

  // Cannot call while holding RenderLock or it will cause a deadlock
  assert(!holdingRenderLock && "Cannot call requestUpdateAndWait() while holding RenderLock");

  xTaskNotify(renderTaskHandle, 1, eIncrement);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

// RenderLock

RenderLock::RenderLock() {
  xSemaphoreTake(activityManager.renderingMutex, portMAX_DELAY);
  isLocked = true;
}

RenderLock::RenderLock([[maybe_unused]] Activity&) {
  xSemaphoreTake(activityManager.renderingMutex, portMAX_DELAY);
  isLocked = true;
}

RenderLock::~RenderLock() {
  if (isLocked) {
    xSemaphoreGive(activityManager.renderingMutex);
    isLocked = false;
  }
}

void RenderLock::unlock() {
  if (isLocked) {
    xSemaphoreGive(activityManager.renderingMutex);
    isLocked = false;
  }
}

/**
 *
 * Checks if renderingMutex is busy.
 *
 * @return true if renderingMutex is busy, otherwise false.
 *
 */
bool RenderLock::peek() { return xQueuePeek(activityManager.renderingMutex, NULL, 0) != pdTRUE; };
