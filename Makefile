# Convenience targets. Everything runs inside .venv, created on first use.
#
#   make test                       unit tests, no hardware needed
#   make test-hw                    end-to-end tests on a connected PBM
#   make test-hw PBM_MOTOR_ID=2     ... against another motor
#   make test ARGS="-k tap -v"      pass extra arguments to pytest

VENV ?= .venv
PYTHON := $(VENV)/bin/python
STAMP := $(VENV)/.installed
ARGS ?=

.DEFAULT_GOAL := help

.PHONY: help install test test-hw test-all build clean distclean

help: ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*## "}; {printf "  %-10s %s\n", $$1, $$2}'

$(STAMP): pyproject.toml
	python3 -m venv $(VENV)
	$(PYTHON) -m pip install -e ".[dev]"
	touch $@

install: $(STAMP) ## Create .venv and install valet-pbm (editable) with dev tools

test: $(STAMP) ## Run unit tests (no hardware needed)
	$(PYTHON) -m pytest $(ARGS)

test-hw: $(STAMP) ## Run end-to-end tests on connected hardware (moves the motor)
	$(PYTHON) -m pytest --hardware -m hardware $(ARGS)

test-all: $(STAMP) ## Run unit and hardware tests
	$(PYTHON) -m pytest --hardware $(ARGS)

build: $(STAMP) ## Build sdist and wheel into dist/
	rm -rf dist
	$(PYTHON) -m build

clean: ## Remove build and test artifacts
	rm -rf build dist src/*.egg-info .pytest_cache
	find . -path ./$(VENV) -prune -o -name __pycache__ -type d -exec rm -rf {} +

distclean: clean ## Also remove .venv
	rm -rf $(VENV)
