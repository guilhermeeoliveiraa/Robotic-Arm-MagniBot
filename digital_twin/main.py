
import pyvista as pv
import numpy as np
import serial
import threading
import queue
import time


# ============================================================
# CONFIGURAÇÕES DA COMUNICAÇÃO SERIAL
# ============================================================

SERIAL_PORT = "COM5"
BAUD_RATE = 115200

UPDATE_INTERVAL = 30


# ============================================================
# CALIBRAÇÃO DOS SERVOS
# ============================================================

SERVO_VALUES_ARE_ABSOLUTE = True

SERVO_NEUTRAL = np.array([
    90,   # Base
    127,  # Ombro
    40,   # Cotovelo
    63,   # Rotação do punho
    81,   # Flexão do punho
    45    # Garra
], dtype=float)


SERVO_OFFSET = np.array([
    0,    # Base
    -40,  # Ombro
    0,    # Cotovelo
    0,    # Rotação do punho
    -70,  # Flexão do punho
    10    # Garra
], dtype=float)


SERVO_DIRECTION = np.array([
    1,  # Base
    1,  # Ombro
    1,  # Cotovelo
    1,  # Rotação do punho
    1,  # Flexão do punho
    -1   # Garra
], dtype=float)


def calibrate_servo_angles(servo_values):

    servo_values = np.asarray(
        servo_values,
        dtype=float
    )

    if servo_values.shape != (6,):
        raise ValueError(
            "São esperados exatamente seis ângulos."
        )

    if SERVO_VALUES_ARE_ABSOLUTE:

        relative_angles = (
            servo_values - SERVO_NEUTRAL
        )

    else:

        relative_angles = servo_values.copy()

    calibrated_angles = (
        relative_angles * SERVO_DIRECTION
        + SERVO_OFFSET
    )

    return calibrated_angles


# ============================================================
# CONFIGURAÇÕES DO SENSOR VL53L0X
# ============================================================

SENSOR_POSITION_LOCAL = np.array([
    -0.068009,
     0.267349,
     0.124147
], dtype=float)


SENSOR_DIRECTION_LOCAL = np.array([
    -0.719312,
    -0.683812,
     0.122438
], dtype=float)


SENSOR_DIRECTION_LOCAL /= np.linalg.norm(
    SENSOR_DIRECTION_LOCAL
)


# Distância recebida em milímetros.
# STL em metros.

DISTANCE_SCALE = 0.001


SENSOR_ARROW_LENGTH = 0.045

SENSOR_MARKER_RADIUS = 0.004
OBJECT_MARKER_RADIUS = 0.004

MAX_OBJECT_POINTS = 3000


# ============================================================
# DADOS GLOBAIS DO SENSOR
# ============================================================

object_points_history = []

sensor_position_global = SENSOR_POSITION_LOCAL.copy()
sensor_direction_global = SENSOR_DIRECTION_LOCAL.copy()

object_distance_mm = 0
object_detected = False


# ============================================================
# CONTROLE DA COMUNICAÇÃO
# ============================================================

serial_queue = queue.Queue(maxsize=1)

serial_connection = None

serial_running = True


# ============================================================
# CARREGAMENTO DO STL
# ============================================================

mesh = pv.read("models/Robotic Arm.stl")

connected = mesh.connectivity()


components = {

    "Base Imóvel": [3],

    "Base Giratória": [6],

    "Braço": [4],

    "Antebraço": [5],

    "Punho Rotação": [8],

    "Punho Flexão": [7],

    "Garra Esquerda": [9, 22],

    "Garra Direita": [10, 21]

}


colors = {

    "Base Imóvel": "#808080",

    "Base Giratória": "#FFD700",

    "Braço": "#FF4500",

    "Antebraço": "#1E90FF",

    "Punho Rotação": "#32CD32",

    "Punho Flexão": "#9370DB",

    "Garra Esquerda": "#FF1493",

    "Garra Direita": "#00CED1"

}


def extract_component(region_ids):

    parts = []

    for region_id in region_ids:

        indices = np.where(
            connected.cell_data["RegionId"] == region_id
        )[0]

        if len(indices) == 0:

            raise ValueError(
                f"Região {region_id} não encontrada no STL."
            )

        part = connected.extract_cells(indices)

        part = part.extract_surface(
            algorithm="dataset_surface"
        ).triangulate()

        parts.append(part)

    return pv.merge(parts)


# ============================================================
# EXTRAÇÃO DOS COMPONENTES
# ============================================================

component_meshes = {}

for name, ids in components.items():

    component_meshes[name] = extract_component(ids)


# ============================================================
# REFERÊNCIAS DOS COMPONENTES
# ============================================================

waist = component_meshes["Base Giratória"]

arm = component_meshes["Braço"]

forearm = component_meshes["Antebraço"]

wrist = component_meshes["Punho Rotação"]

wrist_flex = component_meshes["Punho Flexão"]


# ============================================================
# PIVÔS
# ============================================================

base_pivot = np.array(
    waist.center,
    dtype=float
)


shoulder_pivot = np.array([
    0.027065,
    0.136561,
    0.098319
], dtype=float)


forearm_pivot = np.array([
    0.063202,
    0.248285,
    0.092260
], dtype=float)


wrist_pivot = np.array([
    -0.015312,
     0.289850,
     0.116223
], dtype=float)


wrist_flex_pivot = np.array([
    -0.044577,
     0.302548,
     0.106219
], dtype=float)


left_gripper_pivot = np.array([
    -0.079413,
     0.287467,
     0.128315
], dtype=float)


right_gripper_pivot = np.array([
    -0.083182,
     0.287742,
     0.106459
], dtype=float)


# ============================================================
# EIXOS DAS GARRAS
# ============================================================

left_gripper_axis = np.array([
    -0.672305,
     0.729617,
     0.125156
], dtype=float)


right_gripper_axis = np.array([
     0.672306,
    -0.729616,
    -0.125156
], dtype=float)


print("\n========== GEOMETRIA DO ROBÔ ==========")

print("Centro da base:", base_pivot)

print("Articulação do ombro:", shoulder_pivot)

print("Articulação do cotovelo:", forearm_pivot)

print("Articulação do punho:", wrist_pivot)

print("Articulação da flexão:", wrist_flex_pivot)

print("Pivô da garra esquerda:", left_gripper_pivot)

print("Pivô da garra direita:", right_gripper_pivot)

print("=======================================\n")


# ============================================================
# JANELA PYVISTA
# ============================================================

plotter = pv.Plotter(
    window_size=(1400, 900)
)


actors = {}

display_meshes = {}


for name, part in component_meshes.items():

    display_meshes[name] = part.copy()

    actors[name] = plotter.add_mesh(

        display_meshes[name],

        color=colors[name],

        smooth_shading=True,

        show_edges=False,

        label=name

    )


# ============================================================
# MARCADORES DOS PIVÔS
# ============================================================

def create_pivot_marker(position, color, name):

    marker = pv.Sphere(

        radius=0.005,

        center=position

    )

    actor = plotter.add_mesh(

        marker,

        color=color,

        name=name,

        smooth_shading=True

    )

    return marker, actor


shoulder_marker, shoulder_actor = create_pivot_marker(
    shoulder_pivot,
    "red",
    "Shoulder Pivot"
)


forearm_marker, forearm_actor = create_pivot_marker(
    forearm_pivot,
    "orange",
    "Forearm Pivot"
)


wrist_marker, wrist_actor = create_pivot_marker(
    wrist_pivot,
    "cyan",
    "Wrist Pivot"
)


wrist_flex_marker, wrist_flex_actor = create_pivot_marker(
    wrist_flex_pivot,
    "magenta",
    "Wrist Flex Pivot"
)


left_gripper_marker, left_gripper_actor = create_pivot_marker(
    left_gripper_pivot,
    "yellow",
    "Left Gripper Pivot"
)


right_gripper_marker, right_gripper_actor = create_pivot_marker(
    right_gripper_pivot,
    "white",
    "Right Gripper Pivot"
)


# ============================================================
# SENSOR VL53L0X E OBJETO
# ============================================================

sensor_marker_mesh = pv.Sphere(

    radius=SENSOR_MARKER_RADIUS,

    center=[0, 0, 0]

)


sensor_actor = plotter.add_mesh(

    sensor_marker_mesh,

    color="lime",

    name="VL53L0X Position",

    smooth_shading=True

)


sensor_arrow_actor = None


object_marker_mesh = pv.Sphere(

    radius=OBJECT_MARKER_RADIUS,

    center=[0, 0, 0]

)


object_marker_actor = plotter.add_mesh(

    object_marker_mesh,

    color="blue",

    name="Detected Object",

    smooth_shading=True

)


object_marker_actor.SetVisibility(False)


object_cloud_actor = None

object_cloud = None


# ============================================================
# REFERÊNCIAS ORIGINAIS DAS MALHAS
# ============================================================

original_points = {

    name: part.points.copy()

    for name, part in component_meshes.items()

}


# ============================================================
# GRUPOS DE MOVIMENTO
# ============================================================

base_group = [

    "Base Giratória",

    "Braço",

    "Antebraço",

    "Punho Rotação",

    "Punho Flexão",

    "Garra Esquerda",

    "Garra Direita"

]


arm_group = [

    "Braço",

    "Antebraço",

    "Punho Rotação",

    "Punho Flexão",

    "Garra Esquerda",

    "Garra Direita"

]


forearm_group = [

    "Antebraço",

    "Punho Rotação",

    "Punho Flexão",

    "Garra Esquerda",

    "Garra Direita"

]


wrist_group = [

    "Punho Rotação",

    "Punho Flexão",

    "Garra Esquerda",

    "Garra Direita"

]


wrist_flex_group = [

    "Punho Flexão",

    "Garra Esquerda",

    "Garra Direita"

]


# ============================================================
# MATRIZ DE ROTAÇÃO
# ============================================================

def rotation_matrix(axis, angle):

    axis = np.asarray(
        axis,
        dtype=float
    )

    norm = np.linalg.norm(axis)

    if norm == 0:

        raise ValueError(
            "O eixo de rotação não pode ser zero."
        )

    axis = axis / norm

    x, y, z = axis

    c = np.cos(angle)

    s = np.sin(angle)

    C = 1.0 - c

    return np.array([

        [

            c + x*x*C,

            x*y*C - z*s,

            x*z*C + y*s

        ],

        [

            y*x*C + z*s,

            c + y*y*C,

            y*z*C - x*s

        ],

        [

            z*x*C - y*s,

            z*y*C + x*s,

            c + z*z*C

        ]

    ], dtype=float)


# ============================================================
# ÂNGULOS DAS ARTICULAÇÕES
# ============================================================

base_angle = 0

shoulder_angle = 0

forearm_angle = 0

wrist_angle = 0

wrist_flex_angle = 0

gripper_angle = 0


# ============================================================
# CINEMÁTICA DO SENSOR
# ============================================================

def calculate_sensor_pose():

    # ========================================
    # MATRIZES DE ROTAÇÃO
    # ========================================

    R_base = rotation_matrix(

        [0, 1, 0],

        np.radians(base_angle)

    )


    R_shoulder = rotation_matrix(

        [-0.178803, 0.000000, -0.983885],

        np.radians(shoulder_angle)

    )


    R_forearm = rotation_matrix(

        [-0.178803, 0.000000, -0.983885],

        np.radians(forearm_angle)

    )


    R_wrist = rotation_matrix(

        [0.875964, -0.455352, -0.159190],

        np.radians(wrist_angle)

    )


    R_wrist_flex = rotation_matrix(

        [0.175038, -0.007666, 0.984532],

        np.radians(wrist_flex_angle)

    )


    # ========================================
    # POSIÇÃO DO SENSOR
    # ========================================

    position = SENSOR_POSITION_LOCAL.copy()


    # Flexão do punho.

    position = (

        (position - wrist_flex_pivot)

        @ R_wrist_flex.T

        + wrist_flex_pivot

    )


    # Rotação do punho.

    position = (

        (position - wrist_pivot)

        @ R_wrist.T

        + wrist_pivot

    )


    # Cotovelo.

    position = (

        (position - forearm_pivot)

        @ R_forearm.T

        + forearm_pivot

    )


    # Ombro.

    position = (

        (position - shoulder_pivot)

        @ R_shoulder.T

        + shoulder_pivot

    )


    # Base.

    position = (

        (position - base_pivot)

        @ R_base.T

        + base_pivot

    )


    # ========================================
    # DIREÇÃO DO SENSOR
    # ========================================

    direction = SENSOR_DIRECTION_LOCAL.copy()


    direction = direction @ R_wrist_flex.T

    direction = direction @ R_wrist.T

    direction = direction @ R_forearm.T

    direction = direction @ R_shoulder.T

    direction = direction @ R_base.T


    direction /= np.linalg.norm(direction)


    return position, direction


# ============================================================
# ATUALIZAÇÃO VISUAL DO SENSOR E OBJETO
# ============================================================

def update_sensor_visualization():

    global sensor_position_global

    global sensor_direction_global

    global sensor_arrow_actor


    # Calcula a pose global do sensor.

    sensor_position_global, sensor_direction_global = (

        calculate_sensor_pose()

    )


    # ========================================
    # POSIÇÃO DO SENSOR
    # ========================================

    sensor_actor.SetPosition(

        tuple(sensor_position_global)

    )


    # ========================================
    # DIREÇÃO DO SENSOR
    # ========================================

    if sensor_arrow_actor is not None:

        plotter.remove_actor(

            sensor_arrow_actor,

            render=False

        )


    arrow = pv.Arrow(

        start=sensor_position_global,

        direction=sensor_direction_global,

        scale=SENSOR_ARROW_LENGTH

    )


    sensor_arrow_actor = plotter.add_mesh(

        arrow,

        color="red",

        name="Sensor Direction"

    )


    # ========================================
    # POSIÇÃO DO OBJETO
    # ========================================

    if object_detected and object_distance_mm > 0:

        distance_m = (

            object_distance_mm * DISTANCE_SCALE

        )


        object_position = (

            sensor_position_global

            + sensor_direction_global * distance_m

        )


        object_marker_actor.SetPosition(

            tuple(object_position)

        )


        object_marker_actor.SetVisibility(True)


    else:

        object_marker_actor.SetVisibility(False)


# ============================================================
# ATUALIZAÇÃO DA MONTAGEM
# ============================================================

def update_assembly():

    global base_angle

    global shoulder_angle

    global forearm_angle

    global wrist_angle

    global wrist_flex_angle

    global gripper_angle


    # ========================================
    # MATRIZES DE ROTAÇÃO
    # ========================================

    R_base = rotation_matrix(

        [0, 1, 0],

        np.radians(base_angle)

    )


    R_shoulder = rotation_matrix(

        [-0.178803, 0.000000, -0.983885],

        np.radians(shoulder_angle)

    )


    R_forearm = rotation_matrix(

        [-0.178803, 0.000000, -0.983885],

        np.radians(forearm_angle)

    )


    R_wrist = rotation_matrix(

        [0.875964, -0.455352, -0.159190],

        np.radians(wrist_angle)

    )


    R_wrist_flex = rotation_matrix(

        [0.175038, -0.007666, 0.984532],

        np.radians(wrist_flex_angle)

    )


    R_left_gripper = rotation_matrix(

        left_gripper_axis,

        np.radians(gripper_angle)

    )


    R_right_gripper = rotation_matrix(

        right_gripper_axis,

        np.radians(gripper_angle)

    )


    # ========================================
    # ATUALIZA COMPONENTES
    # ========================================

    for name in components:

        points = original_points[name].copy()


        # Primeiro: movimento das garras.

        if name == "Garra Esquerda":

            points = (

                (points - left_gripper_pivot)

                @ R_left_gripper.T

                + left_gripper_pivot

            )


        elif name == "Garra Direita":

            points = (

                (points - right_gripper_pivot)

                @ R_right_gripper.T

                + right_gripper_pivot

            )


        # Segundo: flexão do punho.

        if name in wrist_flex_group:

            points = (

                (points - wrist_flex_pivot)

                @ R_wrist_flex.T

                + wrist_flex_pivot

            )


        # Terceiro: rotação do punho.

        if name in wrist_group:

            points = (

                (points - wrist_pivot)

                @ R_wrist.T

                + wrist_pivot

            )


        # Quarto: rotação do cotovelo.

        if name in forearm_group:

            points = (

                (points - forearm_pivot)

                @ R_forearm.T

                + forearm_pivot

            )


        # Quinto: rotação do ombro.

        if name in arm_group:

            points = (

                (points - shoulder_pivot)

                @ R_shoulder.T

                + shoulder_pivot

            )


        # Sexto: rotação da base.

        if name in base_group:

            points = (

                (points - base_pivot)

                @ R_base.T

                + base_pivot

            )


        display_meshes[name].points = points

        display_meshes[name].Modified()


    # ========================================
    # MARCADOR DO OMBRO
    # ========================================

    shoulder_position = (

        (shoulder_pivot - base_pivot)

        @ R_base.T

        + base_pivot

    )


    shoulder_marker.points = pv.Sphere(

        radius=0.005,

        center=shoulder_position

    ).points

    shoulder_marker.Modified()


    # ========================================
    # MARCADOR DO COTOVELO
    # ========================================

    elbow_position = (

        (forearm_pivot - shoulder_pivot)

        @ R_shoulder.T

        + shoulder_pivot

    )


    elbow_position = (

        (elbow_position - base_pivot)

        @ R_base.T

        + base_pivot

    )


    forearm_marker.points = pv.Sphere(

        radius=0.005,

        center=elbow_position

    ).points

    forearm_marker.Modified()


    # ========================================
    # MARCADOR DO PUNHO
    # ========================================

    wrist_position = (

        (wrist_pivot - forearm_pivot)

        @ R_forearm.T

        + forearm_pivot

    )


    wrist_position = (

        (wrist_position - shoulder_pivot)

        @ R_shoulder.T

        + shoulder_pivot

    )


    wrist_position = (

        (wrist_position - base_pivot)

        @ R_base.T

        + base_pivot

    )


    wrist_marker.points = pv.Sphere(

        radius=0.005,

        center=wrist_position

    ).points

    wrist_marker.Modified()


    # ========================================
    # MARCADOR DA FLEXÃO
    # ========================================

    flex_position = (

        (wrist_flex_pivot - wrist_pivot)

        @ R_wrist.T

        + wrist_pivot

    )


    flex_position = (

        (flex_position - forearm_pivot)

        @ R_forearm.T

        + forearm_pivot

    )


    flex_position = (

        (flex_position - shoulder_pivot)

        @ R_shoulder.T

        + shoulder_pivot

    )


    flex_position = (

        (flex_position - base_pivot)

        @ R_base.T

        + base_pivot

    )


    wrist_flex_marker.points = pv.Sphere(

        radius=0.005,

        center=flex_position

    ).points

    wrist_flex_marker.Modified()


    # ========================================
    # MARCADOR DA GARRA ESQUERDA
    # ========================================

    left_position = (

        (left_gripper_pivot - wrist_flex_pivot)

        @ R_wrist_flex.T

        + wrist_flex_pivot

    )


    left_position = (

        (left_position - wrist_pivot)

        @ R_wrist.T

        + wrist_pivot

    )


    left_position = (

        (left_position - forearm_pivot)

        @ R_forearm.T

        + forearm_pivot

    )


    left_position = (

        (left_position - shoulder_pivot)

        @ R_shoulder.T

        + shoulder_pivot

    )


    left_position = (

        (left_position - base_pivot)

        @ R_base.T

        + base_pivot

    )


    left_gripper_marker.points = pv.Sphere(

        radius=0.005,

        center=left_position

    ).points

    left_gripper_marker.Modified()


    # ========================================
    # MARCADOR DA GARRA DIREITA
    # ========================================

    right_position = (

        (right_gripper_pivot - wrist_flex_pivot)

        @ R_wrist_flex.T

        + wrist_flex_pivot

    )


    right_position = (

        (right_position - wrist_pivot)

        @ R_wrist.T

        + wrist_pivot

    )


    right_position = (

        (right_position - forearm_pivot)

        @ R_forearm.T

        + forearm_pivot

    )


    right_position = (

        (right_position - shoulder_pivot)

        @ R_shoulder.T

        + shoulder_pivot

    )


    right_position = (

        (right_position - base_pivot)

        @ R_base.T

        + base_pivot

    )


    right_gripper_marker.points = pv.Sphere(

        radius=0.005,

        center=right_position

    ).points

    right_gripper_marker.Modified()


    # ========================================
    # ATUALIZA SENSOR
    # ========================================

    update_sensor_visualization()


    plotter.render()


# ============================================================
# ATUALIZAÇÃO DA NUVEM DE PONTOS
# ============================================================

def update_object_cloud(object_position):

    global object_cloud

    global object_cloud_actor


    # Adiciona nova posição ao histórico.

    object_points_history.append(

        object_position.copy()

    )


    # Limita o histórico.

    if len(object_points_history) > MAX_OBJECT_POINTS:

        object_points_history.pop(0)


    points = np.array(

        object_points_history,

        dtype=float

    )


    # Cria a nuvem na primeira detecção.

    if object_cloud_actor is None:

        object_cloud = pv.PolyData(points)

        number_of_points = len(points)


        object_cloud.verts = np.column_stack([

            np.ones(number_of_points, dtype=np.int64),

            np.arange(number_of_points, dtype=np.int64)

        ]).ravel()


        object_cloud_actor = plotter.add_points(

            object_cloud,

            color="deepskyblue",

            point_size=8,

            render_points_as_spheres=True,

            name="Object Point Cloud"

        )


    else:

        object_cloud.points = points

        number_of_points = len(points)


        object_cloud.verts = np.column_stack([

            np.ones(number_of_points, dtype=np.int64),

            np.arange(number_of_points, dtype=np.int64)

        ]).ravel()


        object_cloud.Modified()


# ============================================================
# LEITURA SERIAL
# ============================================================

def serial_reader():

    global serial_running


    print("\n[UART] Thread serial iniciada.")

    print("[UART] Aguardando dados do ESP32...")

    print("[UART] Protocolo esperado: 7 valores por linha.")


    while serial_running:

        try:

            raw_data = serial_connection.readline()


            if not raw_data:

                continue


            message = raw_data.decode(

                "utf-8",

                errors="replace"

            ).strip()


            if not message:

                continue


            # ========================================
            # SEPARAÇÃO DOS CAMPOS
            # ========================================

            values_text = message.split(",")


            if len(values_text) != 7:

                # Ignora logs do ESP32 e linhas fora do protocolo.

                continue


            # ========================================
            # CONVERSÃO DOS VALORES
            # ========================================

            try:

                values = np.array(

                    [float(value.strip()) for value in values_text],

                    dtype=float

                )

            except ValueError:

                print(

                    "[UART] Linha não numérica ignorada:",

                    repr(message)

                )

                continue


            if not np.all(np.isfinite(values)):

                print(

                    "[UART] Valores inválidos ignorados:",

                    repr(message)

                )

                continue


            # ========================================
            # CALIBRAÇÃO DOS SERVOS
            # ========================================

            calibrated_values = calibrate_servo_angles(

                values[:6]

            )


            # ========================================
            # DISTÂNCIA DO SENSOR
            # ========================================

            distance_mm = int(values[6])


            if distance_mm < 0:

                print(

                    "[VL53L0X] Distância inválida:",

                    distance_mm

                )

                continue


            # ========================================
            # DETECÇÃO
            # ========================================

            # O firmware atual não envia uma flag de detecção.
            #
            # Portanto, consideramos que existe uma detecção
            # quando a distância recebida é positiva.
            #
            # Se o firmware passar a enviar uma flag própria,
            # esta lógica deverá ser substituída.

            detected = distance_mm > 0


            # ========================================
            # PACOTE VÁLIDO
            # ========================================

            print(

                "[UART] PACOTE VÁLIDO:",

                np.array2string(values, precision=2),

                "-> ÂNGULOS DO MODELO:",

                np.array2string(calibrated_values, precision=2)

            )


            print(

                "[VL53L0X] Distância:",

                distance_mm,

                "mm | Detectado:",

                detected

            )


            packet = (

                calibrated_values,

                distance_mm,

                detected

            )


            # ========================================
            # FILA: MANTÉM O PACOTE MAIS RECENTE
            # ========================================

            if serial_queue.full():

                try:

                    serial_queue.get_nowait()

                except queue.Empty:

                    pass


            try:

                serial_queue.put_nowait(packet)

            except queue.Full:

                pass


        except serial.SerialException as error:

            print(

                "[UART] Erro na comunicação serial:",

                error

            )

            serial_running = False

            break


        except Exception as error:

            print(

                "[UART] Erro inesperado:",

                error

            )

            serial_running = False

            break


# ============================================================
# RECEBIMENTO DOS DADOS NO PYVISTA
# ============================================================

def update_from_microcontroller(step):

    global base_angle

    global shoulder_angle

    global forearm_angle

    global wrist_angle

    global wrist_flex_angle

    global gripper_angle

    global object_distance_mm

    global object_detected


    new_data = None


    # Obtém o pacote mais recente.

    while True:

        try:

            new_data = serial_queue.get_nowait()

        except queue.Empty:

            break


    if new_data is None:

        return


    calibrated_values, distance_mm, detected = new_data


    # ========================================
    # ATUALIZAÇÃO DOS ÂNGULOS
    # ========================================

    base_angle = float(calibrated_values[0])

    shoulder_angle = float(calibrated_values[1])

    forearm_angle = float(calibrated_values[2])

    wrist_angle = float(calibrated_values[3])

    wrist_flex_angle = float(calibrated_values[4])

    gripper_angle = float(calibrated_values[5])


    print("\n[PYVISTA] Ângulos atualizados:")

    print("Base:", base_angle)

    print("Ombro:", shoulder_angle)

    print("Cotovelo:", forearm_angle)

    print("Rotação:", wrist_angle)

    print("Flexão:", wrist_flex_angle)

    print("Garra:", gripper_angle)


    # ========================================
    # ATUALIZAÇÃO DO SENSOR
    # ========================================

    object_distance_mm = distance_mm

    object_detected = detected


    # ========================================
    # ATUALIZAÇÃO DA MONTAGEM
    # ========================================

    update_assembly()


    # ========================================
    # CÁLCULO DA POSIÇÃO DO OBJETO
    # ========================================

    if object_detected and object_distance_mm > 0:

        distance_m = (

            object_distance_mm * DISTANCE_SCALE

        )


        object_position = (

            sensor_position_global

            + sensor_direction_global * distance_m

        )


        # Atualiza o marcador.

        object_marker_actor.SetPosition(

            tuple(object_position)

        )

        object_marker_actor.SetVisibility(True)


        # Atualiza a nuvem de pontos.

        update_object_cloud(object_position)


        # ========================================
        # INFORMAÇÕES NO TERMINAL
        # ========================================

        print("\n========== OBJETO DETECTADO ==========")

        print(

            "Distância:",

            object_distance_mm,

            "mm"

        )


        print(

            "Posição global do sensor:",

            sensor_position_global

        )


        print(

            "Direção global do sensor:",

            sensor_direction_global

        )


        print(

            "Posição estimada do objeto:",

            object_position

        )


        print(

            "Quantidade de pontos:",

            len(object_points_history)

        )


        print("======================================")

    else:

        object_marker_actor.SetVisibility(False)

        print("[VL53L0X] Nenhuma detecção válida.")


# ============================================================
# CONEXÃO COM O ESP32
# ============================================================

# IMPORTANTE:
#
# Feche o Monitor Serial do ESP-IDF antes de executar.
#
# A COM5 deve ser utilizada exclusivamente por este programa.
#
# Formato esperado:
#
# 90,127,40,63,81,45,294
#
# Campos:
#
# 0: Base
# 1: Ombro
# 2: Cotovelo
# 3: Rotação do punho
# 4: Flexão do punho
# 5: Garra
# 6: Distância em milímetros


try:

    print("\n========== CONEXÃO SERIAL ==========")

    print("Porta:", SERIAL_PORT)

    print("Baud rate:", BAUD_RATE)


    serial_connection = serial.Serial(

        port=SERIAL_PORT,

        baudrate=BAUD_RATE,

        timeout=0.1

    )


    time.sleep(2)


    serial_connection.reset_input_buffer()


    print("[UART] Porta serial aberta.")

    print("[UART] Aguardando dados do ESP32...")

    print("====================================\n")


except serial.SerialException as error:

    print("\n[ERRO] Não foi possível abrir a porta serial.")

    print("Verifique a porta COM e se ela está sendo utilizada.")

    print("Detalhes:", error)

    raise SystemExit


# ============================================================
# THREAD DE COMUNICAÇÃO
# ============================================================

serial_thread = threading.Thread(

    target=serial_reader,

    daemon=True

)

serial_thread.start()


# ============================================================
# CONFIGURAÇÃO DA VISUALIZAÇÃO
# ============================================================

plotter.add_axes()

plotter.add_legend()

plotter.view_isometric()


# Calcula a posição inicial do sensor.

update_sensor_visualization()


# ============================================================
# EXECUÇÃO
# ============================================================

try:

    # Inicializa a janela gráfica.

    plotter.show(

        interactive_update=True,

        auto_close=False

    )


    print("\n[PYVISTA] Janela inicializada.")

    print("[PYVISTA] Atualização em tempo real iniciada.")


    # ========================================
    # LOOP PRINCIPAL
    # ========================================

    while (

        plotter.iren is not None

        and not plotter.iren.interactor.GetDone()

    ):

        # Processa o pacote mais recente.

        update_from_microcontroller(0)


        # Processa eventos e atualiza a janela.

        plotter.update(

            stime=UPDATE_INTERVAL,

            force_redraw=True

        )


finally:

    # ========================================
    # ENCERRAMENTO DA THREAD
    # ========================================

    print("\n[PYVISTA] Encerrando comunicação...")


    serial_running = False


    if (

        "serial_thread" in globals()

        and serial_thread.is_alive()

    ):

        serial_thread.join(timeout=1.0)


    # ========================================
    # FECHA A PORTA SERIAL
    # ========================================

    if (

        serial_connection is not None

        and serial_connection.is_open

    ):

        serial_connection.close()


    # ========================================
    # FECHA A JANELA
    # ========================================

    plotter.close()


    print("[UART] Porta serial fechada.")

    print("[PYVISTA] Visualização encerrada.")

    print("[SISTEMA] Comunicação serial encerrada.")
