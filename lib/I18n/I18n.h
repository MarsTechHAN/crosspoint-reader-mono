#pragma once

#include <cstdint>

#include "I18nKeys.h"
/**
 * Internationalization (i18n) system for CrossPoint Reader
 */

class I18n {
 public:
  static I18n& getInstance();

  // Disable copy
  I18n(const I18n&) = delete;
  I18n& operator=(const I18n&) = delete;

  // Get localized string by ID
  const char* get(StrId id) const;

  const char* operator[](StrId id) const { return get(id); }

  Language getLanguage() const { return _language; }
  // True when the UI text of this language is written in Han/Kana/Hangul. Those
  // strings draw through the CJK fallback faces, which are registered several
  // points above their Latin counterpart and are correspondingly taller, so the
  // UI has to reserve more vertical room per line. Extend when a translation in
  // another CJK script is added.
  static bool usesCjkScript(const Language lang) { return lang == Language::ZH_CN; }
  bool usesCjkScript() const { return usesCjkScript(_language); }
  void setLanguage(Language lang);
  const char* getLanguageName(Language lang) const;
  static Language languageFromCode(const char* code);

  // Get all unique characters used in a specific language
  // Returns a sorted string of unique characters
  static const char* getCharacterSet(Language lang);

 private:
  I18n() : _language(Language::EN) {}

  Language _language;
};

// Convenience macros
#define tr(id) I18n::getInstance().get(StrId::id)
#define I18N I18n::getInstance()
