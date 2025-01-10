import GUI
import HAL
import numpy as np
import cv2
import time
import random
import os

os.popen("source /opt/ros/humble/setup.bash").read() 

# Скорость
speed = 3
# Нулевые параметры PID
kp, ki, kd = 0.000, 0.0, 0.00

# Лучшая продолжительность до "потери линии" на текущий момент
best_time = 0
best_kp, best_ki, best_kd = kp, ki, kd

# Параметры для поиска новых значений PID
learning_rate = 0.01  # шаг изменения коэффициентов
random_factor = 0.002  # диапазон случайного изменения

# Порог потери линии в пикселях
red_pixel_threshold = 50

# Инициализация переменных для PID
integral = 0
last_error = 0
last_time = None

# Инициализация начальной позиции
i = 0  # Счётчик для контроля первой итерации

# Функция для вычисления ошибки от центра массы красных пикселей
def get_error_from_center_of_mass(masked_image):
    moments = cv2.moments(masked_image, binaryImage=True)
    if moments['m00'] != 0:
        center_x = int(moments['m10'] / moments['m00'])
    else:
        center_x = masked_image.shape[1] // 2
    target_x = masked_image.shape[1] // 2
    error = center_x - target_x
    return error

# Функция для обновления PID
def update_pid(error):
    global integral, last_error, last_time

    current_time = time.time()
    delta_time = current_time - last_time if last_time else 0
    last_time = current_time

    # Пропорциональная составляющая
    p_term = kp * error

    # Интегральная составляющая
    integral += error * delta_time
    i_term = ki * integral

    # Дифференциальная составляющая
    d_term = (kd * (error - last_error) / delta_time) if delta_time > 0 else 0
    last_error = error

    return p_term + i_term + d_term

# Функция для изменения PID параметров, основываясь на результате
def update_pid_parameters(elapsed_time):
    global kp, ki, kd, best_kp, best_ki, best_kd, best_time

    # Если текущее время лучше, обновляем лучшие коэффициенты
    if elapsed_time > best_time:
        best_time = elapsed_time
        best_kp, best_ki, best_kd = kp, ki, kd
        print(f"New best time: {best_time:.2f}s with kp: {best_kp:.5f}, ki: {best_ki:.5f}, kd: {best_kd:.5f}")

    # Регулировка коэффициентов с использованием learning_rate
    randP = random.uniform(-random_factor, random_factor)
    randD = random.uniform(-random_factor, random_factor)
    kp +=  randP
    # ki += learning_rate * (best_ki - ki) + random.uniform(-learning_rate, learning_rate)
    kd += randD
    print(f" best_kp: {best_kp:.5f}, best_kd: {best_kd:.5f}, randp: {randP}, randD: {randD}")

    # Ограничиваем значения, чтобы они оставались положительными
    kp, ki, kd = max(0, kp), max(0, ki), max(0, kd)

# Функция для перемещения робота в начальную позицию
def reset_robot_position():
    HAL.setV(0)  # Останавливаем движение
    HAL.setW(0)  # Останавливаем вращение

    # Перемещаем робота в начальную позицию
    os.popen("ros2 service call /reset_simulation std_srvs/srv/Empty").read()
    # Даем роботу время встать на начальное место
    time.sleep(1)  # Небольшая пауза для стабильности

# Основной цикл
while True:

    # Перемещаем робота на начальную позицию при каждой новой попытке
    if i >= 1:
        reset_robot_position()
    i += 1  # Обновляем счётчик итераций

    # Сбрасываем параметры PID
    integral, last_error, last_time = 0, 0, None  
    start_time = time.time()

    while True:
        HAL.setV(speed)  # Устанавливаем постоянную скорость
        image = HAL.getImage()  # Получаем текущее изображение
        b, g, r = cv2.split(image)  # Извлекаем каналы изображения
        red_mask = cv2.inRange(r, 200, 255)  # Создаём маску для красного цвета
        red_pixel_count = cv2.countNonZero(red_mask)  # Подсчитываем количество красных пикселей
        result = cv2.bitwise_and(image, image, mask=red_mask)
        GUI.showImage(result)  # Отображаем изображение с маской

        # Проверяем, потерял ли робот линию
        elapsed_time = time.time() - start_time
        if red_pixel_count < red_pixel_threshold:
            print(f"Lost line after {elapsed_time:.2f}s")
            update_pid_parameters(elapsed_time)  # Обновляем PID параметры при потере линии
            break  # Выход из вложенного цикла для перезапуска движения

        # Вычисляем ошибку относительно центра линии
        error = get_error_from_center_of_mass(red_mask)

        # Получаем управляющий сигнал от PID
        control_signal = update_pid(error)
        
        # Форматированный вывод значений с ограничением до 5 знаков после запятой

        # Управляем роботом с помощью управляющего сигнала
        HAL.setW(-control_signal)
    # print(f"Error: {error:.5f}, Control Signal: {control_signal:.5f}, kp: {kp:.5f}, ki: {ki:.5f}, kd: {kd:.5f}")
    print(f" kp: {kp:.5f}, ki: {ki:.5f}, kd: {kd:.5f}")