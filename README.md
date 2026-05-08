
---

## Визуальный модуль (Стереокамера + Лазерная триангуляция)

**Назначение:**  
Захват стереопар и вспомогательных данных о глубине для последующей фотограмметрии и построения облака точек объекта.

### Оборудование
- Стереокамера **GXVISION LS2MO1** (USB, разрешение 2560×720)
- Лазерная линия 650 нм, 5 мВт (жёстко закреплена на штоке камеры)
- Платформа: Raspberry Pi 4

### Принцип работы
Данные со стереокамеры используются для фотограмметрической обработки и построения 3D-модели.  
Классическое стереозрение плохо справляется с однородными малотекстурированными поверхностями бетона. Для компенсации применяется **лазерная триангуляция** — лазер проецирует линию на поверхность, по положению которой с точностью до нескольких миллиметров определяется расстояние до объекта. Это значительно повышает плотность и качество итогового облака точек.

### Используемые библиотеки
- `picamera2`
- `opencv-python`
- `numpy`

---

### Захват стереопары (`camera.py`)

```python
from picamera2 import Picamera2
import cv2
import time

class StereoCamera:
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (2560, 720)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2) 

    def capture_stereo_pair(self):

        frame = self.picam2.capture_array()
        left = frame[:, :1280]
        right = frame[:, 1280:]
        return cv2.cvtColor(left, cv2.COLOR_BGR2RGB), cv2.cvtColor(right, cv2.COLOR_BGR2RGB)

    def release(self):
        self.picam2.stop()
```

---

### Лазерная триангуляция (`laser_triangulation.py`)

```python
import cv2
import numpy as np

def extract_laser_line(image):
    
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array([0, 80, 100]), np.array([30, 255, 255]))
    
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def get_laser_points(mask, step=2, min_points=3):
    points = []
    height, width = mask.shape
    
    for x in range(0, width, step):
        col = mask[:, x]
        if np.any(col):
            y = np.argmax(col)                   
            points.append([x, y])
    
    points = np.array(points, dtype=np.float32)
    return points if len(points) >= min_points else None


def laser_triangulation(laser_points_2d, fx, fy, cx, cy, laser_plane_params):
    if laser_points_2d is None or len(laser_points_2d) == 0:
        return None
    
    points_3d = []
    
    for px, py in laser_points_2d:
        x = (px - cx) / fx
        y = (py - cy) / fy
        z = 1.0
        
        ray = np.array([x, y, z])
        ray = ray / np.linalg.norm(ray)
        
        a, b, c, d = laser_plane_params
        denominator = np.dot([a, b, c], ray)
        
        if abs(denominator) < 1e-6:
            continue
            
        t = -d / denominator
        if t > 0:
            point_3d = ray * t
            points_3d.append(point_3d)
    
    return np.array(points_3d, dtype=np.float32)
```

---

**Важно:**  
Для работы функции `laser_triangulation()` необходимо предварительно откалибровать параметры камеры (`fx, fy, cx, cy`) и плоскости лазера (`laser_plane_params`). Эти значения будут сохранены в конфигурационном файле после калибровки.

Модуль обеспечивает захват стереопар и генерацию 3D-точек через лазерную триангуляцию. Полная фотограмметрическая обработка выполняется на следующих этапах.

---

### Архитектура системы

Робототехнический комплекс мультимодальной дефектоскопии построен по **двухуровневой архитектуре** (Low-level + High-level). Такое разделение обеспечивает надёжность, реальное время на нижнем уровне и удобную высокоуровневую обработку данных.

#### 1. Нижний уровень (Low-level) — **ESP32-S3**

Отвечает за все операции, требующие жёсткого реального времени и прямого управления периферией.

**Основные функции ESP32-S3:**
- Управление бесколлекторным **импеллером** 120 Вт (создание вакуумного прижима к поверхности)
- Управление двигателями платформы + сбор **одометрии**
- Управление **ударно-акустическим модулем (УАМ)** — соленоид и микрофон
- Сбор данных с **датчика влажности**
- Приём данных с **энкодера лебёдки** (высота робота) по радиоканалу
- Радиосвязь с **пультом управления** по **NRF24L01**
- Связь с Raspberry Pi по **UART**
- Watchdog и безопасное отключение импеллера при потере связи

#### 2. Верхний уровень (High-level) — **Raspberry Pi 4**

Выполняет роль центрального «дирижёра» и edge-компьютера.

**Основные функции Raspberry Pi 4:**
- Оркестрация всей системы и управление ESP32 по UART
- Управление стереокамерой
- Выполнение **лазерной триангуляции** и обработка карты глубины
- Агрегация и синхронизация данных со всех датчиков (видео, глубина, УАМ, влажность, одометрия, высота)
- Запись полного датасета на **внешний SSD-диск**
- Предобработка данных перед передачей на вычислительный компьютер

#### Связь между уровнями

Raspberry Pi и ESP32-S3 общаются по **UART** (115200 бод) с помощью собственного **бинарного протокола** (COBS-фрейминг + CRC-16).  
Протокол поддерживает команды от верхнего уровня и периодическую телеметрию от нижнего.

#### Пульт управления и демонстрационный режим

Для удобства отладки, презентаций и работы без Raspberry Pi предусмотрен **демонстрационный режим**.

**В этом режиме:**
- Raspberry Pi полностью отключена
- Всем роботом управляет **пульт дистанционного управления** по радиоканалу **NRF24L01**
- Пульт позволяет:
  - Включать / выключать импеллер
  - Выполнять одиночные и серийные удары УАМ
  - Управлять движением платформы
  - Получать базовую телеметрию

#### Режимы работы

| Режим                  | Raspberry Pi | Способ управления                    | Основное применение                  |
|------------------------|--------------|--------------------------------------|--------------------------------------|
| **Полный**             | Подключена   | Raspberry Pi → UART → ESP32          | Сбор данных, полевые испытания       |
| **Демонстрационный**   | Отсутствует  | Пульт → NRF24L01 → ESP32             | Выставки, презентации, отладка       |




---

## Модуль Ударно-Акустической Дефектоскопии (УАМ)

**УАМ** — специализированный модуль для неразрушающего контроля бетонных конструкций ударно-акустическим методом. Позволяет выявлять внутренние дефекты (пустоты, расслоения, коррозию арматуры) по спектральному отпечатку колебаний после контролируемого механического удара.

Существует два исполнения модуля:
- Встраиваемый вариант (для робота)
- **Портативный комплектный модуль серии УДАРНИК АКУСТИЧЕСКОГО ТРУДА** — для сбора обучающего датасета

### Аппаратная часть

- **Микроконтроллер**: ESP32-S3
- **Соленоид**: 12 В (питается от 14 В)
- **Управление соленоидом**: электромагнитное реле
- **Датчик**: PZT-диск
- **Предусилитель**: комплектный, на основе платы MAX953 
- **АЦП**: PCM1808 (I2S)
- **Управление**: Кнопка удара + кнопка разметки дефекта (удерживается оператором)

**Параметры оцифровки:**
- Частота дискретизации: 44100 Гц
- Разрядность: 16 бит
- Длительность записи одного удара: 100 мс

### Пайплайн выполнения одного удара

1. Оператор плотно прижимает модуль к поверхности бетона.
2. При наличии дефекта (по визуальной оценке) удерживает кнопку разметки дефекта.
3. Нажимает кнопку удара.
4. Реле активирует соленоид → производится удар.
5. Задержка 10 мс (для затухания переходных процессов).
6. Запускается запись сигнала через PCM1808 в течение 100 мс.
7. Данные по Wi-Fi передаются на телефон оператора.
8. На телефоне файл автоматически сохраняется в формате `.wav` с префиксом `healthy_` или `defective_`.
9. После завершения сбора все файлы переносятся с телефона на компьютер для обработки.

### Прошивка ESP32-S3

```cpp
#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>

const int RELAY_PIN = 5;
const int DEFECT_BTN = 6;
const int STRIKE_BTN = 7;


const int I2S_BCK = 26;   
const int I2S_WS  = 25;
const int I2S_DIN = 33;

bool defectMode = false;

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(DEFECT_BTN, INPUT_PULLUP);
  pinMode(STRIKE_BTN, INPUT_PULLUP);
  
  WiFi.begin("SSID", "PASSWORD");
  

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void loop() {
  if (digitalRead(DEFECT_BTN) == LOW) defectMode = true;
  
  if (digitalRead(STRIKE_BTN) == LOW) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(50);
    digitalWrite(RELAY_PIN, LOW);
    delay(10);
    recordAndSendSignal();
    defectMode = false;
    delay(600);
  }
}

void recordAndSendSignal() {
  int16_t samples[4410];
  size_t bytesRead;
  
  i2s_read(I2S_NUM_0, samples, sizeof(samples), &bytesRead, portMAX_DELAY);
  
  String prefix = defectMode ? "defective" : "healthy";
  sendDataToPhone(prefix, samples, 4410);
}
```

### Обработка сигнала и FFT

После переноса данных на компьютер выполняется предобработка и спектральный анализ.

```python
import numpy as np
from scipy.fft import rfft, rfftfreq
import soundfile as sf
import os

def process_file(filepath):
    data, sr = sf.read(filepath)
    

    window = np.hanning(len(data))      
    windowed = data * window
    
    
    N = len(windowed)
    yf = rfft(windowed)
    xf = rfftfreq(N, 1 / sr)
    spectrum = np.abs(yf)
    

    processed_path = filepath.replace("raw", "processed").replace(".wav", ".npy")
    os.makedirs(os.path.dirname(processed_path), exist_ok=True)
    np.save(processed_path, np.array([xf, spectrum]))
    
    return xf, spectrum
```

**Особенности спектрального анализа:**
- **Алгоритм**: Cooley-Tukey (Fast Fourier Transform) — классический быстрый алгоритм вычисления Дискретного преобразования Фурье.
- **Реализация**: `scipy.fft.rfft` — оптимизированная версия для реальных сигналов (использует симметрию и вычисляет только положительные частоты).
- **Окно**: Hann (Hanning) — эффективно снижает спектральную утечку и уровень боковых лепестков.
- **Частотное разрешение**: ≈ 10 Гц, что оптимально для анализа характерных резонансных частот бетонных конструкций.

### Сбор датасета

Для обучения мультимодальной нейронной сети используется **портативный комплектный модуль серии УДАРНИК АКУСТИЧЕСКОГО ТРУДА**.

Оператор вручную обследует разные участки бетонных конструкций, размечает дефекты с помощью боковой кнопки и производит удары. Данные в реальном времени передаются по Wi-Fi на телефон, а затем загружаются на ПК.

**Собранный датасет:**
- Общее количество ударов: ~500
- Healthy (здоровый бетон): ~300
- Defective (дефектный бетон): ~200

Все файлы сохраняются в формате **`.wav`** (44100 Гц, 16-bit).

### Скрипты обработки

- `process_signal.py` — обработка одного файла (FFT + сохранение спектра)

---
