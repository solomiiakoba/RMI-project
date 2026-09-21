"""
c1_line_follower.py

Passo 1: Seguimento de linha usando os 3 ground sensors (GS0, GS1, GS2)
com controlo proporcional, preparado para curvas de 90 graus (formato "Г").

Esta versao NAO trata ainda:
  - deteccao de checkpoints via camara (Passo 2)
  - correcao de sentido (Passo 3)
  - recuperacao apos perder a linha (Passo 4)
Esses blocos serao adicionados a seguir e integrados no loop principal.
"""

from controller import Robot

# ---------------------------------------------------------------------------
# Inicializacao
# ---------------------------------------------------------------------------
robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# Velocidade base (quando a linha esta centrada)
cruiseVelocity = 3.0
maxVelocity = 6.0  # limite fisico do motor do e-puck (ajustar se necessario)

# Motores
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(cruiseVelocity)
rightMotor.setVelocity(cruiseVelocity)

# Ground sensors: GS0 = esquerda, GS1 = centro, GS2 = direita
ground_sensors = [robot.getDevice('gs' + str(x)) for x in range(3)]
for gs in ground_sensors:
    gs.enable(timeStep)

# Camara (ainda nao usada neste passo, mas ja fica ligada para o Passo 2)
camera = robot.getDevice("camera")
camera.enable(timeStep)

# ---------------------------------------------------------------------------
# Calibracao dos sensores de chao
# ---------------------------------------------------------------------------
# Estes valores devem ser AFINADOS experimentalmente no Webots:
# imprime ground_sensor_values durante um teste e ve os valores reais
# sobre branco e sobre a linha preta.
WHITE_VALUE = 820   # valor tipico sobre fundo branco (medido: 760-830)
BLACK_VALUE = 320   # valor tipico sobre a linha preta (medido: 300-350)

# Ganho do controlo proporcional (regime normal, curvas suaves / retas).
KP = 0.02

# A partir deste valor de |erro|, entramos em modo de viragem apertada
# (necessario para os cantos de 90 graus em "Г", onde a correcao proporcional
# normal e demasiado lenta e perde a linha).
SHARP_TURN_THRESHOLD = 0.35

# Velocidades usadas no modo de viragem apertada (pivot turn).
# A roda interior roda bastante mais devagar (ou ao contrario) do que a
# exterior, para rodar o robo no proprio eixo em vez de so desviar a trajetoria.
SHARP_TURN_OUTER_SPEED = 4.0
SHARP_TURN_INNER_SPEED = -1.0  # negativo = roda anda ligeiramente para tras

# Abaixo deste valor, nenhum sensor esta a ver a linha com confianca real
# (valores mais baixos do que isto sao apenas ruido do sensor sobre branco).
LINE_PRESENT_THRESHOLD = 0.30

# Peso de cada sensor no calculo do erro (esquerda negativo, direita positivo)
# GS0 (esquerda) = -1, GS1 (centro) = 0, GS2 (direita) = +1
SENSOR_WEIGHTS = [-1.0, 0.0, 1.0]


def normalize(value):
    """
    Converte o valor bruto do sensor para um peso entre 0 (branco) e 1 (preto).
    Valores fora do intervalo calibrado sao truncados (clamped).
    """
    v = (WHITE_VALUE - value) / (WHITE_VALUE - BLACK_VALUE)
    return max(0.0, min(1.0, v))


def compute_error(raw_values):
    """
    Calcula o erro de posicao da linha em relacao ao centro do robo.
    Erro negativo -> linha esta a esquerda -> robo deve virar a esquerda.
    Erro positivo -> linha esta a direita -> robo deve virar a direita.
    Erro perto de 0 -> linha centrada.

    Se nenhum sensor detetar preto com confianca, devolve None (linha perdida).

    NOTA IMPORTANTE: usamos o valor MAXIMO dos pesos (nao a soma) para decidir
    se a linha esta realmente presente. A soma dos 3 sensores sobre fundo
    branco puro ja da um valor residual (~0.3) so por ruido do sensor, o que
    fazia o codigo anterior "pensar" que via a linha quando na verdade
    ja a tinha perdido (foi o que aconteceu na curva em Г).
    """
    weights = [normalize(v) for v in raw_values]

    if max(weights) < LINE_PRESENT_THRESHOLD:
        # nenhum sensor individual ve preto com confianca -> linha perdida
        return None

    total_weight = sum(weights)
    # media ponderada das posicoes dos sensores, pesada pela "escuridao" de cada um
    error = sum(w * sw for w, sw in zip(weights, SENSOR_WEIGHTS)) / total_weight
    return error


def clamp(value, low, high):
    return max(low, min(high, value))


# ---------------------------------------------------------------------------
# Loop principal
# ---------------------------------------------------------------------------
while robot.step(timeStep) != -1:

    ground_sensor_values = [g.getValue() for g in ground_sensors]
    error = compute_error(ground_sensor_values)

    if error is None:
        # Linha perdida: por agora, comportamento provisorio (parar).
        # No Passo 4 substituimos isto por uma manobra de recuperacao real.
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        print("LINHA PERDIDA - valores:", ground_sensor_values)
        continue

    if abs(error) >= SHARP_TURN_THRESHOLD:
        # MODO DE VIRAGEM APERTADA: o erro e grande de mais para o controlo
        # proporcional normal conseguir acompanhar a curva a tempo (e o que
        # acontecia nos cantos em "Г" - a linha saia de baixo do robo antes
        # de a correcao suave completar a viragem). Aqui fazemos um pivot
        # mais decisivo: uma roda quase para (ou anda para tras), a outra
        # mantem-se rapida, para rodar o robo no proprio eixo.
        if error < 0:
            # linha a esquerda -> pivotar para a esquerda
            left_speed = SHARP_TURN_INNER_SPEED
            right_speed = SHARP_TURN_OUTER_SPEED
        else:
            # linha a direita -> pivotar para a direita
            left_speed = SHARP_TURN_OUTER_SPEED
            right_speed = SHARP_TURN_INNER_SPEED
    else:
        # Controlo proporcional normal: quanto maior o erro, maior a correcao.
        # Se erro > 0 (linha a direita): reduz velocidade direita, aumenta esquerda.
        # Se erro < 0 (linha a esquerda): reduz velocidade esquerda, aumenta direita.
        correction = KP * error * 100  # escala o erro (entre -1 e 1) para algo util
        left_speed = cruiseVelocity + correction
        right_speed = cruiseVelocity - correction

    left_speed = clamp(left_speed, -maxVelocity, maxVelocity)
    right_speed = clamp(right_speed, -maxVelocity, maxVelocity)

    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

    # Debug (comentar/remover depois de afinar os parametros)
    print(f"gs={ground_sensor_values}  error={error:.2f}  L={left_speed:.2f}  R={right_speed:.2f}")
