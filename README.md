# Projeto_RMA

Neste repositório se encontra o código utilizado para resolver o problema proposto na disciplina de RMA.

# Reconhecimento e créditos

Este código se baseou no algoritmo RRT do nimRobotics, disponível [neste repositório](https://github.com/nimRobotics/RRT).

# Instalação

Primeiro passo é a criação do workspace:

```sh
mkdir -p ~/workspace_projrma/src
cd ~/workspace_projrma/
catkin_make
source devel/setup.bash
catkin init
catkin clean
```
O próximo é a instalação dos pacotes relacionados ao código e ao ros

```sh
cd ~/workspace_projrma/src
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/mateusjp/Projeto_RMA.git
catkin build
cd smb_common
```

Lista de bibliotecas necessárias para o *Python*:

* cv2
* imutils
* numpy
* math
* random

```sh
pip3 install pip opencv-python imutils numpy
```

# Execução

Execute o arquivo *rrt_projeto.launch* que se encontra na pasta **workspace/src/smb_common/smb_gazebo/launch**

```sh
cd workspace/src/smb_common/smb_gazebo/launch
roslaunch rrt_projeto.launch
```

Execute o arquivo *projeto_rma.py*

```sh
python3 projeto_rma.py
```

O programa irá iniciar e encontrar tentará encontrar o caminho para o objetivo, pode acontecer um erro no processo, basta executar o código novamente.

# Resultado

O vídeo do resultado pode ser visto [neste link](https://drive.google.com/file/d/1ls4PEDdtmtQigpjfPncIXWGmtyRwSLwk/view?usp=sharing)
