.PHONY:
	install
	run


ENV_DIR = mav_venv
BIN     = $(ENV_DIR)/bin
DEP     = ../dependencies.txt
PYTHON  = python3.9


install:            ## Install the project
	python3.9 -m venv mav_venv
	$(BIN)/pip install --upgrade pip
	$(BIN)/pip install -f requirements.txt -e .

clean:              ## Remove the project and its dependencies from your python3.9 environment. It also removes any downloaded cached files
	$(BIN)/pip uninstall -y $(ENV_DIR)
	$(BIN)/pip cache purge

run:
	$(BIN)/python3.9 ./book_assignments/mavsim_chap2.py
