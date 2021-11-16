#!/bin/bash
(trap 'kill 0' SIGINT; python3 main.py & python3 plot_graph.py)
