#!/bin/bash
(trap 'kill 0' SIGINT; python3 main_interface.py & python3 main_plot_graph.py)
