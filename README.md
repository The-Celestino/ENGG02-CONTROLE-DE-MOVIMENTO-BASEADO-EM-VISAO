<div align="center">
  <h1 style="font-family: 'Orbitron', sans-serif; font-size: 42px; color: #0a84ff; letter-spacing: 2px;">🤖 Controle de Movimento Baseado em Visão para Robô UR5 🚀</h1>
</div>

<p align="center">
  <img src="https://img.shields.io/badge/ROS-Noetic-blue?style=for-the-badge&logo=ros"/>
  <img src="https://img.shields.io/badge/Ubuntu-20.04-orange?style=for-the-badge&logo=ubuntu"/>
  <img src="https://img.shields.io/badge/Simulação-Gazebo-brightgreen?style=for-the-badge&logo=oculus"/>
  <img src="https://img.shields.io/badge/Visão-OpenCV-informational?style=for-the-badge&logo=opencv"/>
</p>

<div align="center">
  <img src="https://media.giphy.com/media/UTq4Jp3qBuZKo/giphy.gif" width="500"/>
</div>

---

## 📜 Índice

* [Visão Geral do Projeto](#-visão-geral-do-projeto)
* [Demonstração](#-demonstração)
* [Arquitetura do Sistema e Lógica de Funcionamento](#-arquitetura-do-sistema-e-lógica-de-funcionamento)
* [Tecnologias Utilizadas](#️-tecnologias-e-ferramentas)
* [Estrutura do Workspace](#-estrutura-do-workspace)
* [Pré-requisitos](#️-pré-requisitos)
* [Guia de Instalação](#-instalação-e-configuração)
* [Como Executar a Simulação](#️-executando-a-simulação)

---

## 📖 Visão Geral do Projeto

🎯 Este projeto apresenta um sistema de controle para um braço robótico **Universal Robots UR5**, simulado no ambiente **Gazebo**, que realiza tarefas de manipulação de objetos de forma autônoma através de **percepção visual em tempo real**.

🔍 Equipado com uma garra Robotiq 85 e uma câmera RGB, o robô utiliza a biblioteca **OpenCV** para identificar, localizar e interagir com caixas coloridas dispostas em seu ambiente de trabalho.

🤖 A análise visual com OpenCV é o núcleo do sistema: por meio da câmera, as caixas são detectadas dinamicamente e o robô adapta seu movimento às posições reais dos objetos, mesmo que variem entre execuções. Essa inteligência visual elimina a necessidade de coordenadas fixas, permitindo uma operação flexível e robusta.

🔧 O sistema combina a stack de navegação e planejamento do **ROS (Robot Operating System)** com **MoveIt!**, que garante trajetórias seguras e livres de colisão. A integração entre visão computacional e controle robótico cria um pipeline completo que vai da percepção à ação.

---

### 🏗️ Arquitetura do Sistema e Lógica de Funcionamento

O sistema funciona através da coordenação de vários nós ROS, formando um pipeline de Percepção -> Decisão -> Ação.

📸 **Nó de Percepção Visual (`camera.py`)**

* Recebe frames do tópico da câmera RGB (`/camera/rgb/image_raw`).
* Processa a imagem com OpenCV (HSV, máscaras de cor, contornos).
* Publica as coordenadas dos centroides das caixas em `/box_coordinates`.

🧠 **Nó de Orquestração e Controle (`ativar_tracking.py`)**

* Recebe as coordenadas do nó de visão.
* Converte as coordenadas 2D em posições 3D.
* Envia metas de pose para o MoveIt! planejar e executar.

🦾 **Planejamento com MoveIt!**

* Gera trajetórias seguras e livres de colisão com base nas metas.

🏗️ **Execução no Gazebo**

* O robô executa os movimentos com `ros_control`.
* A garra utiliza o plugin `gazebo_ros_link_attacher` para simular a pega dos objetos.

---

### 🛠️ Tecnologias e Ferramentas

* 🧠 **ROS Noetic**
* 🌌 **Gazebo 11**
* 🎯 **MoveIt!**
* 📷 **OpenCV**
* 🤖 **UR5 e Robotiq 85 (URDF)**
* 🐍 **Python 3**
* 🧰 **Catkin Tools**

---

### 📂 Estrutura do Workspace

```bash
ur5_ws/
├── src/
│   ├── ur_description/
│   ├── robotiq_gripper/
│   ├── ur5_gripper_moveit_config/
│   ├── ur5_seletor_caixa/
│   └── ...
```

---

### ⚙️ Pré-requisitos

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-moveit python3-catkin-tools
sudo apt install ros-noetic-universal-robot ros-noetic-gazebo-ros-control ros-noetic-ros-controllers
sudo apt install python3-opencv
```

---

### 🚀 Guia de Instalação

```bash
mkdir -p ~/ur5_ws/src
cd ~/ur5_ws/src
git clone https://github.com/the-celestino/engg02-controle-de-movimento-baseado-em-visao.git
cd ~/ur5_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
echo "source ~/ur5_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### ▶️ Executando a Simulação

🖥️ **Terminal 1: Lançar o ambiente do Gazebo**

```bash
roslaunch ur5_gripper_moveit_config versao_final.launch
```

🎯 **Terminal 2: Ativar scripts de visão e controle**

```bash
roslaunch ur5_seletor_caixa ativar_tracking.launch
```

📦 O robô detectará automaticamente as caixas e realizará a manipulação conforme as posições detectadas pela câmera.

---

<div align="center">
  <img src="https://i.imgur.com/simulacaoTech.gif" alt="Simulação UR5" width="700">
  <br><br>
  <img src="https://img.shields.io/badge/Feito%20com-OpenCV%20+%20ROS%20+%20Gazebo-000000?style=for-the-badge&logo=codeforces">
</div>

---

## ⚠️ Disclaimer

> ⚠️ **A instalação do ROS e seus pré-requisitos pode variar entre sistemas e versões.**
>
> É fundamental estar atento ao log de instalação de cada etapa e verificar mensagens de erro ou dependências ausentes antes de prosseguir.

---

<style>
@import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@700&display=swap');
</style>
