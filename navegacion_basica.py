from controller import Robot, GPS, DistanceSensor, Gyro, Motor

# Inicialización del robot y configuración inicial
robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# Configuración de los motores de las ruedas
wheel_left = Motor("wheel1 motor")
wheel_right = Motor("wheel2 motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

# Configuración del GPS
gps = GPS("gps")
gps.enable(timeStep)

# Configuración de los sensores de distancia
ds_front = DistanceSensor("distance sensor1")
ds_front.enable(timeStep)

ds_left = DistanceSensor("distance sensor2")
ds_left.enable(timeStep)

ds_right = DistanceSensor("distance sensor3")
ds_right.enable(timeStep)

# Configuración del giroscopio
gyro = Gyro("gyro")
gyro.enable(timeStep)

# Variables para la navegación y control del robot
vel_avance = 6.28
vel_retroceso = -6.28
vel_giro = 1.0  # Velocidad de giro (ajustar según sea necesario)

estado = "avanzar"
cnt_pasos = 0
pasos_por_baldosa = 0
baldosa_inicial = None

baldosa_inicial_coords = None
distancia_tolerancia = 0.1  

# Función para imprimir telemetría de los sensores
def print_sensor_telemetry():
    pos = gps.getValues()
    gyro_angle = gyro.getValues()[1]  # Se asume que el ángulo relevante es en el eje Y
    distancia_frontal = ds_front.getValue()
    distancia_izquierda = ds_left.getValue()
    distancia_derecha = ds_right.getValue()
    
    print(f"GPS Position: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f}")
    print(f"Gyro Angle: {gyro_angle:.2f} degrees")
    print(f"Distance Sensor - Front: {distancia_frontal:.2f} units")
    print(f"Distance Sensor - Left: {distancia_izquierda:.2f} units")
    print(f"Distance Sensor - Right: {distancia_derecha:.2f} units")
    print(f"Current State: {estado}")
    print("---------------------------")

# Función para avanzar recto
def avanzar():
    wheel_left.setVelocity(vel_avance)
    wheel_right.setVelocity(vel_avance)

# Función para retroceder
def retroceder():
    wheel_left.setVelocity(vel_retroceso)
    wheel_right.setVelocity(vel_retroceso)

# Función para girar a la izquierda
def girar_izquierda():
    wheel_left.setVelocity(-vel_giro)
    wheel_right.setVelocity(vel_giro)

# Función para girar a la derecha
def girar_derecha():
    wheel_left.setVelocity(vel_giro)
    wheel_right.setVelocity(-vel_giro)

# Bucle principal de control
while robot.step(timeStep) != -1:
    pos = gps.getValues()
    distancia_frontal = ds_front.getValue()
    distancia_izquierda = ds_left.getValue()
    distancia_derecha = ds_right.getValue()
    
    print_sensor_telemetry()
    
    if estado == "avanzar":
        if distancia_frontal > 0.1:
            avanzar()
        else:
            retroceder()
            estado = "retroceder"

    elif estado == "retroceder":
        if distancia_izquierda > 0.2:
            girar_izquierda()
        elif distancia_derecha > 0.2:
            girar_derecha()
            estado = "girando_derecha"
        else:
            avanzar()
            estado = "avanzar"

    elif estado == "girando_derecha":
        if distancia_izquierda > 0.1:
            girar_izquierda()
        elif distancia_derecha > 0.1:
            girar_derecha()
        else:
            avanzar()
            estado = "avanzar"

    # Verificar si el robot ha vuelto a la baldosa inicial
    if (baldosa_inicial_coords is None or
        (abs(pos[0] - baldosa_inicial_coords[0]) < distancia_tolerancia and
         abs(pos[2] - baldosa_inicial_coords[1]) < distancia_tolerancia)):
        baldosa_inicial_coords = pos[0], pos[2]

# Detener los motores al finalizar
wheel_left.setVelocity(0)
wheel_right.setVelocity(0)
