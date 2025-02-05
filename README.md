# Robotic-Arm

## Repositório de Código para Transformações, Cinemática e Trajetórias

Este repositório contém uma implementação modular de transformações homogêneas 2D, cálculos de cinemática direta e inversa, e geração de trajetórias para robótica. O código foi organizado em classes e arquivos separados para facilitar a manutenção e reutilização.

## Estrutura do Repositório

O repositório está organizado da seguinte forma:

- `transformations.py`: Contém a classe `SE2` que lida com transformações homogêneas 2D.
- `kinematics.py`: Contém as classes `ForwardKinematics` e `InverseKinematics` para cálculos de cinemática direta e inversa.
- `trajectories.py`: Contém as classes `JointTrajectory` e `EuclideanTrajectory` para cálculos e plotagem de trajetórias.
- `main.py`: Arquivo principal que chama as classes e funções para exemplificar o uso.

## Como Rodar o Código

Para rodar o código, siga os passos abaixo:

1. Clone o repositório:

   ```bash
   git clone https://github.com/ldcnetto/Robotic-Arm.git
   cd Robotic-Arm
   ```

2. Instale as dependências:

   Certifique-se de ter o Python instalado. As principais bibliotecas utilizadas são `numpy` e `matplotlib`. Você pode instalá-las usando `pip`:

   ```bash
   pip install numpy matplotlib
   ```

3. Execute o arquivo principal:

   ```bash
   python main.py
   ```

## Lógica do Código

### Transformações Homogêneas 2D (`transformations.py`)

A classe `SE2` contém métodos para criar matrizes de transformação homogênea 2D. Essas matrizes são usadas para transformar coordenadas de um sistema de referência para outro.

- `SE2_generic(x, y, theta)`: Cria uma matriz de transformação homogênea 2D com translação `(x, y)` e rotação `theta`.
- `SE2_xy(x, y)`: Cria uma matriz de transformação homogênea 2D com translação `(x, y)` e sem rotação.
- `SE2_theta(theta)`: Cria uma matriz de transformação homogênea 2D com rotação `theta` e sem translação.
- `transform(input, x, y, theta, rotate_first=False)`: Aplica uma transformação homogênea 2D a um ponto de entrada.

### Cinemática Direta e Inversa (`kinematics.py`)

As classes `ForwardKinematics` e `InverseKinematics` lidam com cálculos de cinemática direta e inversa, respectivamente.

- `ForwardKinematics.fk(theta1, theta2)`: Calcula a posição final de um braço robótico com dois ângulos de junta `theta1` e `theta2`.
- `InverseKinematics.ik(x, y)`: Calcula os ângulos de junta `theta1` e `theta2` necessários para alcançar uma posição `(x, y)`.

### Trajetórias (`trajectories.py`)

As classes `JointTrajectory` e `EuclideanTrajectory` lidam com a geração e plotagem de trajetórias.

- `JointTrajectory.traj_joint(theta1_init, theta2_init, theta1_final, theta2_final, ts=0.1)`: Gera e plota a trajetória das juntas do robô.
- `EuclideanTrajectory.plot_trajs(traj_1, traj_2)`: Gera e plota a trajetória Euclidiana.

### Arquivo Principal (`main.py`)

O arquivo `main.py` serve como ponto de entrada para testar e usar as funcionalidades implementadas. Ele contém exemplos de uso das classes e funções.
