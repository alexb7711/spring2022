.PHONY:
	setup
	install
	clean
	run
	test


ENV_DIR = mav_venv
BIN     = $(ENV_DIR)/bin
DEP     = ../dependencies.txt
PYTHON  = python

install:
	ipython kernel install --name "mav-venv" --user

uninstall:
	jupyter kernelspec uninstall "mav-venv"

setup:            ## Install the project
	python -m venv mav_venv
	$(BIN)/pip install --upgrade pip
	$(BIN)/pip install -f requirements.txt -e .

clean:              ## Remove the project and its dependencies from your python environment. It also removes any downloaded cached files
	$(BIN)/pip uninstall -y $(ENV_DIR)
	$(BIN)/pip cache purge

run:
	$(BIN)/python ./book_assignments/mavsim_chap11.py

test:
	$(BIN)/python ./mav_sim/unit_tests/ch11b_dubins_path_manager_test.py
