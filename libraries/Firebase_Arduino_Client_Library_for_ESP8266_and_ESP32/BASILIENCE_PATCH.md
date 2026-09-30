# Basilience patch to Firebase Arduino Client Library 4.4.17

This folder is a vendored copy of "Firebase Arduino Client Library for ESP8266
and ESP32" version 4.4.17 with **three one-line default changes** so that one
Firebase connection attempt can never hold the Basilience main loop for more
than about 5 seconds. Nothing else in the library was changed.

## Why

In upstream 4.4.17 no Firebase setting reaches the two waits below. Both apply
to every connection the library opens, RTDB and authentication/token refresh
alike, because both build their socket and TLS engine from the same two classes.

| Phase | Upstream | Basilience | Where it lives |
|---|---|---|---|
| TCP connect | 30000 ms | 2000 ms | `WiFiClientImpl::_timeout` default. The library never calls `setTimeout()` on it. |
| TLS handshake | 60000 ms | 3000 ms | `BSSL_SSL_Client::_handshake_timeout` default. The library never calls `setHandshakeTimeout()`. `FirebaseConfig::timeout.sslHandshake` exists but is never read. |

TCP connect plus TLS handshake is therefore at most 5000 ms. The Basilience
firmware enforces that with a `static_assert` in `FirebaseManager.h`.

Not changed, on purpose:

- **DNS** is lwIP's own resolver retry schedule. No Firebase or client setting
  covers it.
- **Server response** is already bounded by `config.timeout.serverResponse`,
  which the firmware sets to 4000 ms and which works as documented.
- **`config.timeout.socketConnection`** has a unit bug (the library passes
  milliseconds to a setter that expects seconds) but the value is overwritten
  before it can matter, so it is left alone and the firmware does not set it.

## How the firmware notices a missing patch

The two changed headers define the macros
`BASILIENCE_FIREBASE_TCP_CONNECT_TIMEOUT_MS` and
`BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS`. `FirebaseManager.h` stops the
build with `#error` if either is missing, so compiling against an unpatched
copy (for example the arduino-cli user library folder) cannot happen silently.
The boot summary prints the values that were actually compiled in.

## Patch

Generated with `git diff --relative` from this folder. The files use CRLF line
endings.

```diff
diff --git a/src/client/SSLClient/client/BSSL_SSL_Client.h b/src/client/SSLClient/client/BSSL_SSL_Client.h
--- a/src/client/SSLClient/client/BSSL_SSL_Client.h
+++ b/src/client/SSLClient/client/BSSL_SSL_Client.h
@@ -40,6 +40,15 @@
 #include "../ESP_SSLClient_FS.h"
 #include "../ESP_SSLClient_Const.h"
 
+// BASILIENCE PATCH 2/3 (see BASILIENCE_PATCH.md in the library root).
+// TLS handshake timeout, in milliseconds, for EVERY connection this library
+// opens. Upstream default: 60000, and nothing in the library ever calls
+// setHandshakeTimeout() (FirebaseConfig::timeout.sslHandshake exists but is
+// never read in 4.4.17). Also a marker, like the TCP one in WiFiClientImpl.h.
+#ifndef BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS
+#define BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS 3000
+#endif
+
 #if defined(USE_EMBED_SSL_ENGINE) && !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_NANO_RP2040_CONNECT)
 #define EMBED_SSL_ENGINE_BASE_OVERRIDE override
 #else
@@ -368,7 +377,7 @@ private:
     size_t _recvapp_len;
     // Renameing from _timeout which also defined in parent's Stream class.
     unsigned long _timeout_ms = 15000;
-    unsigned long _handshake_timeout = 60000;
+    unsigned long _handshake_timeout = BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS; // BASILIENCE PATCH 2/3 (upstream: 60000)
     unsigned long _tcp_session_timeout = 0;
     unsigned long _session_ts = 0;
     bool _isSSLEnabled = false;
diff --git a/src/client/SSLClient/client/BSSL_TCP_Client.h b/src/client/SSLClient/client/BSSL_TCP_Client.h
--- a/src/client/SSLClient/client/BSSL_TCP_Client.h
+++ b/src/client/SSLClient/client/BSSL_TCP_Client.h
@@ -441,7 +441,7 @@ private:
     Client *_basic_client = nullptr;
     // Renameing from _timeout which also defined in parent's Stream class.
     unsigned long _timeout_ms = 15000;
-    unsigned long _handshake_timeout = 60000;
+    unsigned long _handshake_timeout = BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS; // BASILIENCE PATCH 3/3 (upstream: 60000; only re-applied by operator=)
     unsigned long _tcp_session_timeout = 0;
 
     char *mStreamLoad(Stream &stream, size_t size);
diff --git a/src/client/WiFiClientImpl.h b/src/client/WiFiClientImpl.h
--- a/src/client/WiFiClientImpl.h
+++ b/src/client/WiFiClientImpl.h
@@ -33,6 +33,16 @@
 #if !defined(WIFICLIENT_IMPL_H) && defined(ESP32)
 #define WIFICLIENT_IMPL_H
 
+// BASILIENCE PATCH 1/3 (see BASILIENCE_PATCH.md in the library root).
+// TCP connect timeout, in milliseconds, for EVERY connection this library
+// opens - RTDB and authentication/token refresh both create their socket from
+// this class and nothing in the library ever overrides it. Upstream default:
+// 30000. Also serves as the marker that lets Basilience's FirebaseManager.h
+// refuse to build against an unpatched copy of the library.
+#ifndef BASILIENCE_FIREBASE_TCP_CONNECT_TIMEOUT_MS
+#define BASILIENCE_FIREBASE_TCP_CONNECT_TIMEOUT_MS 2000
+#endif
+
 #include <lwip/sockets.h>
 class WiFiClientImpl : public Client
 {
@@ -249,7 +259,7 @@ public:
 
 private:
     int _socket = -1;
-    int _timeout = 30000;
+    int _timeout = BASILIENCE_FIREBASE_TCP_CONNECT_TIMEOUT_MS; // BASILIENCE PATCH 1/3 (upstream: 30000)
     size_t _rxBuffSize = 2048;
     uint8_t *_rxBuff = nullptr;
     size_t _fillPos = 0;
```

## Reapplying to a fresh copy of 4.4.17

Save the diff above as `firebase_timeout.patch` and run this from the library
folder. It was checked to apply cleanly to an untouched 4.4.17 copy.

```
git apply --ignore-whitespace firebase_timeout.patch
```

Then confirm three lines mention `BASILIENCE_FIREBASE` in `WiFiClientImpl.h` and
`BSSL_SSL_Client.h`, and one in `BSSL_TCP_Client.h`. Building the firmware
without the patch fails with the `#error` in `FirebaseManager.h`.

## Upgrading the library

A newer version may fix the underlying problem or move these lines. Do not
upgrade without rereading `WiFiClientImpl.h` (`_timeout`) and
`BSSL_SSL_Client.h` (`_handshake_timeout`), and repeat the checks the firmware
depends on: nothing calls `setTimeout()` on the basic client, and nothing calls
`setHandshakeTimeout()` on the SSL client.

## Other copies of this library

Only this copy is patched. The arduino-cli user-library folder
(`C:\Users\jakej\ArduinoCLIData\user\libraries`) has its own untouched copy.
The Arduino IDE uses this repository's `libraries` folder as its sketchbook, so
it builds against this patched copy.
