#pragma once
#include <Arduino.h>
#include <SPI.h>

class EInkDisplay {
 public:
  // Constructor with pin configuration
  EInkDisplay(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy);

  // Destructor — frees heap-allocated frame buffers if still live.
  ~EInkDisplay() {
    free(frameBuffer0);
    free(frameBuffer1);
  }

  // Refresh modes (guarded to avoid redefinition in test builds)
  enum RefreshMode {
    FULL_REFRESH,  // Full refresh with complete waveform
    HALF_REFRESH,  // Half refresh (1720ms) - balanced quality and speed
    FAST_REFRESH   // Fast refresh using custom LUT
  };

  // Set X3 panel geometry and mode (must be called before begin())
  void setDisplayX3();

  // Returns true when running in X3 mode (set by setDisplayX3()).
  bool isX3Mode() const { return _x3Mode; }

  // Initialize the display hardware and driver
  void begin();

  // Legacy compile-time dimensions kept for compatibility.
  static constexpr uint16_t DISPLAY_WIDTH = 800;
  static constexpr uint16_t DISPLAY_HEIGHT = 480;
  static constexpr uint16_t DISPLAY_WIDTH_BYTES = DISPLAY_WIDTH / 8;
  static constexpr uint32_t BUFFER_SIZE = DISPLAY_WIDTH_BYTES * DISPLAY_HEIGHT;
  static constexpr uint16_t X3_DISPLAY_WIDTH = 792;
  static constexpr uint16_t X3_DISPLAY_HEIGHT = 528;
  static constexpr uint16_t X3_DISPLAY_WIDTH_BYTES = X3_DISPLAY_WIDTH / 8;
  static constexpr uint32_t X3_BUFFER_SIZE = X3_DISPLAY_WIDTH_BYTES * X3_DISPLAY_HEIGHT;
  static constexpr uint32_t MAX_BUFFER_SIZE = 52272;  // max(800x480, 792x528) / 8

  // Runtime dimensions
  uint16_t getDisplayWidth() const { return displayWidth; }
  uint16_t getDisplayHeight() const { return displayHeight; }
  uint16_t getDisplayWidthBytes() const { return displayWidthBytes; }
  uint32_t getBufferSize() const { return bufferSize; }

  // Frame buffer operations
  void clearScreen(uint8_t color = 0xFF) const;
  void drawImage(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                 bool fromProgmem = false) const;
  void drawImageTransparent(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            bool fromProgmem = false) const;
  void swapBuffers();
  // Copy the just-displayed frame (frameBufferActive) back into the write
  // buffer. displayBuffer()/triggerDisplay() end with swapBuffers(), so the
  // write buffer otherwise holds the frame from two refreshes ago — callers
  // that patch a few regions and re-display (instead of re-rendering the full
  // frame) must call this first. No-op in single-buffer mode, where the write
  // buffer is already the displayed frame.
  void syncWriteBufferFromActive() const;
  void setFramebuffer(const uint8_t* bwBuffer) const;

  void copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer);
  void copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer);
  void copyGrayscaleMsbBuffers(const uint8_t* msbBuffer);

  void cleanupGrayscaleBuffers(const uint8_t* bwBuffer);
  void cleanupGrayscaleWithPreviousBuffer();
  void syncRedRamFromFrameBuffer();

  void displayBuffer(RefreshMode mode = FAST_REFRESH, bool turnOffScreen = false);
  // EXPERIMENTAL: Windowed update - display only a rectangular region
  void displayWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool turnOffScreen = false);
  void displayGrayBuffer(bool turnOffScreen = false, const unsigned char* lut = nullptr, bool factoryMode = false);

  // Select the X3 grayscale LUT bank for differential AA refreshes.
  //  false (default): OEM V5.6.21 _gc bank (~2.4 s panel time, accurate grays)
  //  true:            community 7-frame _grayscale bank (~130 ms panel time,
  //                   mid-tones run slightly darker than OEM; matches what
  //                   papyrix-reader has shipped since 2025-11)
  // No effect on X4 — its single `lut_grayscale` already runs at ~500 ms.
  // Callers can flip this between page renders.
  void setFastGrayscaleLut(bool fast) { _x3FastGrayLut = fast; }
  bool getFastGrayscaleLut() const { return _x3FastGrayLut; }

  void refreshDisplay(RefreshMode mode = FAST_REFRESH, bool turnOffScreen = false);

  // Hint the X3 policy to run a one-shot full resync on next update.
  void requestResync(uint8_t settlePasses = 0);

  // Zero the X3 initial-full-sync counter and mark the RED RAM as already
  // synced. Call after a warm restart (the panel was active ms ago); without
  // this, the first two paints after begin() are promoted to FULL (~770ms
  // each) regardless of the requested mode.
  void skipInitialResync();

  // debug function
  void grayscaleRevert();

  // Arm / disarm the BUSY GPIO ISR for a single waveform-completion event.
  // armBusyIsr() must be called immediately before issuing CMD_DISPLAY_REFRESH
  // or CMD_X3_DISPLAY_REFRESH. waitForRefresh() then sleeps on the semaphore
  // instead of spin-polling, freeing the CPU for the loop task.
  void armBusyIsr();
  void disarmBusyIsr();

  // LUT control
  void setCustomLUT(bool enabled, const unsigned char* lutData = nullptr);

  // Power management
  void deepSleep();

  // Access to frame buffer
  uint8_t* getFrameBuffer() const { return frameBuffer; }

  // Release both frame buffers back to the heap. Call only after the final
  // displayBuffer() — the e-ink controller retains the image in its own RAM.
  // After this call no display operations may be performed until begin() is
  // called again. Intended for the web server session where the device reboots
  // on exit and ~52KB of heap is more valuable than rendering ability.
  // Calling when already released is a safe no-op.
  void releaseBuffers() {
    free(frameBuffer0);
    frameBuffer0 = nullptr;
    frameBuffer = nullptr;
    free(frameBuffer1);
    frameBuffer1 = nullptr;
    frameBufferActive = nullptr;
  }

  // Non-blocking display split:
  //
  // triggerDisplay() sends all pixel data to the controller via SPI, issues
  // the refresh command, performs swapBuffers() and any pre-waveform DTM1
  // pre-sync, then RETURNS IMMEDIATELY without waiting for the waveform.
  //
  // completeDisplay() blocks (via FreeRTOS semaphore sleep) until the BUSY
  // pin deasserts, then performs all post-waveform work (conditioning passes,
  // flag updates). It must be called on the SAME TASK as triggerDisplay().
  //
  // displayBuffer() = triggerDisplay() + completeDisplay() (backwards-compat).
  //
  // Concurrency contract:
  //   - Both methods must be called from the same task (the render task).
  //   - No other task may call any display SPI method between triggerDisplay()
  //     and completeDisplay() — the SPI bus is logically owned by the render
  //     task for this entire window.
  //   - frameBuffer is safe to overwrite after triggerDisplay() returns.
  void triggerDisplay(RefreshMode mode, bool turnOffScreen = false);
  void completeDisplay();
  bool isRefreshPending() const { return _refreshPending; }
  // X4: true when the controller's RED RAM holds the last-displayed BW frame
  // (set by syncRedRamFromFrameBuffer / cleanupGrayscaleBuffers, cleared by grayscale
  // waveforms and non-fast refreshes). When true, reallocSecondaryBuffer() does not
  // invalidate the differential baseline — the host copy is stale but the controller's
  // RED RAM is unaffected by host-side allocation.
  bool isRedRamSynced() const { return !_x3Mode && _redRamSynced; }

  // Release only the secondary (previous-frame) buffer (~52 KB on X3, ~48 KB
  // on X4) to free heap temporarily — e.g. during chapter compilation when
  // no rendering is happening. BW display continues to work; fast differential
  // refresh degrades to half/full on X4 until reallocSecondaryBuffer() is
  // called. On X3 fast differential is unaffected (previous-frame lives in
  // the controller's DTM1 RAM). Grayscale AA is unavailable until restored.
  // No-op if already released. Returns true if the buffer was freed.
  //
  // IMPORTANT: swapBuffers() alternates which of frameBuffer0/frameBuffer1 is
  // the active write target. We must always free the INACTIVE buffer
  // (frameBufferActive), never the active one (frameBuffer). Freeing
  // frameBuffer would leave the write target dangling — causing silent heap
  // corruption on every subsequent pixel write.
  bool releaseSecondaryBuffer() {
    if (!frameBufferActive) return false;
    // Identify which named slot holds the secondary buffer, then free it.
    // frameBuffer is the active write target; frameBufferActive is the one
    // to free. After swapBuffers() the slot assignments can be either way.
    if (frameBufferActive == frameBuffer0) {
      free(frameBuffer0);
      frameBuffer0 = nullptr;
    } else {
      free(frameBuffer1);
      frameBuffer1 = nullptr;
    }
    frameBufferActive = nullptr;
    return true;
  }

  // Reallocate the secondary buffer after releaseSecondaryBuffer().
  // Initialises it to white (0xFF) and reseeds frameBufferActive.
  // Returns true on success; false if malloc fails (buffer stays null).
  bool reallocSecondaryBuffer() {
    if (frameBufferActive) return true;  // already allocated
    // Allocate into whichever named slot was freed; prefer the one that is
    // currently nullptr so we never leak a live buffer.
    uint8_t** slot = (frameBuffer0 == nullptr) ? &frameBuffer0 : &frameBuffer1;
    *slot = static_cast<uint8_t*>(malloc(bufferSize));
    if (!*slot) return false;
    frameBufferActive = *slot;
    memset(frameBufferActive, 0xFF, bufferSize);
    return true;
  }

  // Returns true if the secondary buffer is currently allocated.
  bool hasSecondaryBuffer() const { return frameBufferActive != nullptr; }

  // Opt in to single-buffer fast differential refresh (X4 only). When the
  // secondary (previous-frame) buffer has been released, a FAST refresh
  // normally downgrades to HALF because the host no longer holds the previous
  // frame to re-seed RED RAM with. But the controller retains the last
  // displayed frame in RED RAM, and syncRedRamFromFrameBuffer() keeps it
  // current after every refresh — so a differential only needs the new frame
  // written to BW RAM. Enable this when the caller can guarantee RED RAM still
  // holds the previously displayed frame: i.e. the secondary was released right
  // after a normal refresh and only plain BW updates happen until it is
  // restored. No effect on X3 (fast differential there already works without
  // the secondary buffer) and no effect while the secondary buffer is present.
  void setSingleBufferFastDiff(bool enabled) { _singleBufferFastDiff = enabled; }
  bool singleBufferFastDiff() const { return _singleBufferFastDiff; }

  // Save the current framebuffer to a PBM file (desktop/test builds only)
  void saveFrameBufferAsPBM(const char* filename);

 private:
  // Internal geometry setter used by setDisplayX3().
  void setDisplayDimensions(uint16_t width, uint16_t height);

  // Pin configuration
  int8_t _sclk, _mosi, _cs, _dc, _rst, _busy;

  // Runtime display geometry
  uint16_t displayWidth = DISPLAY_WIDTH;
  uint16_t displayHeight = DISPLAY_HEIGHT;
  uint16_t displayWidthBytes = DISPLAY_WIDTH_BYTES;
  uint32_t bufferSize = BUFFER_SIZE;
  bool _x3Mode = false;
  bool _x3RedRamSynced = false;
  // X4: allow fast differential against the controller's retained RED-RAM
  // baseline when the secondary buffer is released. See setSingleBufferFastDiff.
  bool _singleBufferFastDiff = false;
  // X4: RED RAM holds the last-displayed frame (set by syncRedRamFromFrameBuffer,
  // cleared when grayscale or non-fast waveforms leave RED RAM in an unknown state).
  bool _redRamSynced = false;
  struct X3GrayState {
    bool lastBaseWasPartial = false;
    bool lsbValid = false;
  };
  X3GrayState _x3GrayState;
  uint8_t _x3InitialFullSyncsRemaining = 0;
  bool _x3ForceFullSyncNext = false;
  uint8_t _x3ForcedConditionPassesNext = 0;
  // When true, X3 differential grayscale uses the 7-frame community
  // `lut_x3_*_grayscale` bank instead of the OEM 53-frame `lut_x3_*_gc` bank.
  // See setFastGrayscaleLut() for trade-offs.
  bool _x3FastGrayLut = false;
  // Frame buffers — heap-allocated in begin(), freed by releaseBuffers().
  uint8_t* frameBuffer0 = nullptr;
  uint8_t* frameBuffer = nullptr;
  uint8_t* frameBuffer1 = nullptr;
  uint8_t* frameBufferActive = nullptr;

  // Non-blocking refresh state (triggerDisplay / completeDisplay split).
  bool _refreshPending = false;     // true between trigger and complete
  bool _refreshTurnOff = false;     // turnOffScreen arg stored for complete
  bool _refreshDoFullSync = false;  // X3: doFullSync stored for complete
  bool _refreshDoHalfSync = false;  // X3: doHalfSync stored for complete

  // SPI settings
  SPISettings spiSettings;

  // State
  bool isScreenOn = false;
  bool customLutActive = false;
  bool inGrayscaleMode = false;
  bool drawGrayscale = false;

  // Low-level display control
  void resetDisplay();
  void sendCommand(uint8_t command);
  void sendData(uint8_t data);
  void sendData(const uint8_t* data, uint16_t length);
  void waitForRefresh(const char* comment = nullptr);
  void waitWhileBusy(const char* comment = nullptr);
  // Shared body for the two waits above. X4 (SSD1677) and X3 (UC81xx-class)
  // use opposite BUSY-line polarities:
  //   X4: active HIGH. BUSY HIGH while working, drops LOW when done.
  //   X3: active LOW.  BUSY HIGH when idle, drops LOW while working, returns
  //                    HIGH when done.
  // The per-panel polling logic therefore stays gated; consolidation here
  // is the function body only.
  void pollBusy(const char* comment, const char* completeWord);
  void initDisplayController();

  // Low-level display operations
  void setRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void writeRamBuffer(uint8_t ramBuffer, const uint8_t* data, uint32_t size);

  // X3 (UC81xx) primitives. Promoted from inline lambdas that used to be
  // redefined in displayBuffer / displayGrayBuffer / grayscaleRevert. These
  // fuse a command byte and a short data payload into one CS-low SPI
  // transaction — used for LUT register / mode-select / partial-window
  // writes where the payload is small. Bulk plane writes go through
  // sendPlaneX3/fillPlaneX3 (separated sendCommand+sendData). Not an
  // atomicity requirement, just convenience.
  void sendCommandDataX3(uint8_t cmd, const uint8_t* data, uint16_t len);
  void sendCommandDataByteX3(uint8_t cmd, uint8_t d0);
  void sendCommandDataByteX3(uint8_t cmd, uint8_t d0, uint8_t d1);
  // Bulk-write a pixel plane to one of the DTM RAM commands. Y-flips rows
  // in-place (X3 controller scans gates upward), optionally inverts bits
  // before sending, then restores the buffer.
  void sendPlaneX3(uint8_t ramCmd, uint8_t* buf, bool invert);
  // Fill an entire RAM plane with a single byte (e.g., 0xFF for white).
  // Streams a small row buffer repeatedly so the framebuffer isn't touched.
  void fillPlaneX3(uint8_t ramCmd, uint8_t fillByte);
  // Load all 5 LUT registers (VCOM/WW/BW/WB/BB) in one call. Each pointer
  // must reference a 42-byte LUT bank in PROGMEM/DRAM.
  void loadLutBankX3(const uint8_t* vcom, const uint8_t* ww, const uint8_t* bw, const uint8_t* wb, const uint8_t* bb);
  void loadLutBankX3WithCdi(uint8_t cdi0, const uint8_t* vcom, const uint8_t* ww, const uint8_t* bw, const uint8_t* wb,
                            const uint8_t* bb);
  void loadLutBankX3WithCdi(uint8_t cdi0, uint8_t cdi1, const uint8_t* vcom, const uint8_t* ww, const uint8_t* bw,
                            const uint8_t* wb, const uint8_t* bb);
  // Power-on if needed, trigger refresh, optionally power-off. The `tag`
  // string is included verbatim in busy-wait log lines.
  void triggerRefreshX3(bool turnOffScreen, const char* tag);
};

// Factory LUTs extracted from firmware V3.1.9_CH_X4_0117.bin.
// Uses absolute 2-bit pixel encoding for single-pass grayscale refresh.
// See EInkDisplay.cpp for encoding details.
extern const unsigned char lut_factory_fast[];     // 110 bytes, 60 frames, FR=0x44
extern const unsigned char lut_factory_quality[];  // 110 bytes, 50 frames, FR=0x22
