#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include "SD_MMC.h"

// =====================
// VARIÁVEIS DE CONTROLE
// =====================
int totalChunks = 0;
int currentChunk = 0;
bool imageComplete = false;

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow Receptor Final");

  // Inicializa o cartão SD
  if (!SD_MMC.begin()) {
    Serial.println("Falha ao montar o cartão SD");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("Nenhum cartão SD detectado");
    return;
  }

  Serial.print("Tipo de cartão SD: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("DESCONHECIDO");

  // Modo AP
  WiFi.mode(WIFI_AP);
  configDeviceAP();

  Serial.print("MAC AP: ");
  Serial.println(WiFi.softAPmacAddress());

  // Inicializa ESP-NOW
  InitESPNow();

  // Registra callback de recepção
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (imageComplete) {
    imageComplete = false;

    File file = SD_MMC.open("/imagem_recebida.jpg");
    if (!file) {
      Serial.println("Erro ao abrir a imagem");
    } else {
      Serial.printf("Imagem salva com sucesso! Tamanho: %d bytes\n", file.size());
      file.close();
    }
  }
}

// =====================
// CALLBACK DE RECEPÇÃO
// =====================
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
  Serial.printf("Dados recebidos de %s - Tamanho: %d bytes\n", macStr, data_len);

  // Processa os dados
  processReceivedData(data, data_len);
}

// =====================
// PROCESSAMENTO DOS DADOS
// =====================
void processReceivedData(const uint8_t *data, int data_len) {
  uint8_t header = *data++;
  data_len--;

  switch (header) {
    case 0x01: // Cabeçalho de início da transmissão
      Serial.println("Início da transmissão de imagem");
      currentChunk = 0;
      totalChunks = (data[0] << 8) | data[1];
      data += 2;
      data_len -= 2;

      Serial.println("Total de pacotes: " + String(totalChunks));

      if (SD_MMC.exists("/imagem_recebida.jpg")) {
        SD_MMC.remove("/imagem_recebida.jpg");
      }
      break;

    case 0x02: // Dados do pacote
      currentChunk = (data[0] << 8) | data[1];
      data += 2;
      data_len -= 2;

      Serial.println("Recebendo chunk: " + String(currentChunk));

      File file = SD_MMC.open("/imagem_recebida.jpg", FILE_APPEND);
      if (!file) {
        Serial.println("Erro ao abrir arquivo para escrita");
      } else {
        file.write(data, data_len);
        file.close();
      }

      if (currentChunk == totalChunks) {
        imageComplete = true;
        Serial.println("Imagem recebida por completo!");
      }
      break;
  }
}

// =====================
// INICIALIZAÇÃO DO ESP-NOW
// =====================
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW inicializado com sucesso");
  } else {
    Serial.println("Falha na inicialização do ESP-NOW");
    ESP.restart();
  }
}

// =====================
// CONFIGURA MODO AP
// =====================
void configDeviceAP() {
  const char *SSID = "ReceptorFinal";
  bool result = WiFi.softAP(SSID, "senha_receptor", 1, 0);
  if (!result) {
    Serial.println("Falha na configuração do AP");
  } else {
    Serial.println("AP configurado. Nome: " + String(SSID));
  }
}
