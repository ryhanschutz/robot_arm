"""Inicia o simulador diretamente no Modo Desafio (Loop infinito: pegar a bolinha e soltar no chão).

Você pode alternar para o Modo Manual a qualquer momento clicando no botão do painel lateral
ou pressionando a tecla [L] na janela 3D do MuJoCo.

Uso:
  python sim/desafio_bolinha.py
"""
import sys
from simulador import main

if __name__ == "__main__":
    if "--desafio" not in sys.argv:
        sys.argv.insert(1, "--desafio")
    main()

