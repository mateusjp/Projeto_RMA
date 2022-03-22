# Projeto_RMA

Neste repositório se encontra o código utilizado para resolver o problema proposto na disciplina de RMA.

# Reconhecimento e créditos

Este código se baseou no algoritmo RRT do nimRobotics, disponível [neste repositório](https://github.com/nimRobotics/RRT).

# Instalação

Para a execução será necessário do arquivo **smb_gazebo**, disponível na *.zip* da entrega da atividade ou [neste link](https://drive.google.com/file/d/1WESPXsfFjpNs6oRHgiX93BinENzkG6cw/view?usp=sharing). Essa pasta deve ser descompactada em **workspace/src**

```sh
cd workspace/src
```

Lista de bibliotecas necessárias para o *Python*:

* cv2
* imutils
* numpy
* math
* random

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
