#include <WiFi.h>
#include "VideoStream.h"
#include "AmebaFatFS.h"
#include "NNObjectDetection.h"
#include "BLEDevice.h"
#include "RTSP.h"
#include "StreamIO.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"

// Configuración común
#define CHANNEL         0
#define CHANNELNN       3
#define NNWIDTH        576
#define NNHEIGHT       320
#define BLE_INTERVAL   10000  // 10 segundos entre envios BLE
#define IMG_INTERVAL    3000  // 3 segundos entre capturas
#define MAX_QUEUE       100   // Máximo eventos en cola offline

// BLE
#define UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_RX           "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_TX           "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Objetos globales
VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;
RTSP rtsp;
AmebaFatFS fs;
BLEService UartSrv(UART_SERVICE_UUID);
BLECharacteristic Rx(UUID_RX), Tx(UUID_TX);

// Estados
struct Evento {
    String nombre;
    String imagen;
};
Queue<Evento> eventosOffline(MAX_QUEUE);
uint32_t imgCount = 0;
bool bleConectado = false;
unsigned long lastBleSend = 0;

// ─────────────────────────── Setup ───────────────────────────
void setup() {
    Serial.begin(115200);
    
    // 1. Inicializar WiFi
    WiFi.begin("A55 de Juan", "12345678");
    while (WiFi.status() != WL_CONNECTED) delay(1000);
    
    // 2. Inicializar cámara y NN
    Camera.configVideoChannel(CHANNEL, config);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();
    ObjDet.configVideo(configNN);
    ObjDet.modelSelect(OBJECT_DETECTION, CUSTOMIZED_YOLOV4TINY);
    ObjDet.begin();
    
    // 3. Inicializar microSD
    if (!fs.begin()) Serial.println("Error microSD");
    
    // 4. Configurar BLE
    BLE.beginPeripheral();
    BLE.addService(UartSrv);
    BLE.setAdvertName("AMB82-DriverMonitor");
    
    // 5. Iniciar streams
    Camera.channelBegin(CHANNEL);
    Camera.channelBegin(CHANNELNN);
    rtsp.begin();
}

// ─────────────────────── Funciones útiles ─────────────────────
void guardarEvento(const String& evento) {
    // Capturar y guardar imagen
    uint32_t imgAddr, imgLen;
    Camera.getImage(CHANNEL, &imgAddr, &imgLen);
    
    String filename = String(fs.getRootPath()) + "event_" + imgCount + ".jpg";
    File file = fs.open(filename.c_str());
    if (file) {
        file.write((uint8_t*)imgAddr, imgLen);
        file.close();
        
        // Agregar a cola
        eventosOffline.push(Evento{evento, filename});
        imgCount++;
    }
}

void enviarPorBLE(const Evento& evento) {
    String json = "{\"evento\":\"" + evento.nombre + "\",\"imagen\":\"" + evento.imagen + "\"}";
    Tx.writeString(json.c_str());
    if (bleConectado) Tx.notify();
}

// ─────────────────────────── Loop ────────────────────────────
void loop() {
    // 1. Verificar conexión BLE
    bleConectado = BLE.connected(0);
    
    // 2. Procesar detecciones
    auto resultados = ObjDet.getResult();
    for (auto& res : resultados) {
        String clase = itemList[res.type()].objectName;
        
        if (clase == "somnolencia" || clase == "distraccion" || clase == "celular") {
            guardarEvento(clase);
        }
    }
    
    // 3. Enviar eventos pendientes
    if (millis() - lastBleSend > BLE_INTERVAL) {
        while (!eventosOffline.empty()) {
            Evento e = eventosOffline.pop();
            if (bleConectado) {
                enviarPorBLE(e);
            } else {
                // Guardar en archivo de log
                File log = fs.open("pendientes.txt", FILE_APPEND);
                log.println(e.nombre + "," + e.imagen);
                log.close();
            }
        }
        lastBleSend = millis();
    }
    
    delay(100);  // Reducir carga CPU
}