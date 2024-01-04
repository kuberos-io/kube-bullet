
import sys
import argparse
from loguru import logger

from kube_bullet_simulator import KubeBulletSimulator


def main():
    
    logger.remove()
    logger.add(sys.stdout, level="DEBUG")

    simulator = KubeBulletSimulator()
    simulator.spin()


if __name__ == "__main__":
    main()    
