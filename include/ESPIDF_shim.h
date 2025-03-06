// A thin layer bridging API gaps with newer ESP IDF.

#ifndef ESP_RETURN_ON_ERROR
#ifdef NDEBUG
#define ESP_RETURN_ON_ERROR(x) \
  do {                         \
    esp_err_t __err_rc = (x);  \
    if (__err_rc != ESP_OK) {  \
      return __err_rc;         \
    }                          \
  } while (0)
#else
#define ESP_RETURN_ON_ERROR(x)                                                   \
  do {                                                                           \
    esp_err_t __err_rc = (x);                                                    \
    if (__err_rc != ESP_OK) {                                                    \
      ESP_LOGW(TAG, "ESP Error (%s:%d) 0x%x", __ESP_FILE__, __LINE__, __err_rc); \
      return __err_rc;                                                           \
    }                                                                            \
  } while (0)
#endif  // NDEBUG
#endif  // ESP_RETURN_ON_ERROR

#ifndef ESP_GOTO_ON_ERROR
#ifdef NDEBUG
#define ESP_GOTO_ON_ERROR(x, goto_tag) \
  do {                                 \
    esp_err_t __err_rc = (x);          \
    if (__err_rc != ESP_OK) {          \
      goto goto_tag;                   \
    }                                  \
  } while (0)
#else
#define ESP_GOTO_ON_ERROR(x, goto_tag)                                           \
  do {                                                                           \
    esp_err_t __err_rc = (x);                                                    \
    if (__err_rc != ESP_OK) {                                                    \
      ESP_LOGW(TAG, "ESP Error (%s:%d) 0x%x", __ESP_FILE__, __LINE__, __err_rc); \
      goto goto_tag;                                                             \
    }                                                                            \
  } while (0)
#endif  // NDEBUG
#endif  // ESP_GOTO_ON_ERROR