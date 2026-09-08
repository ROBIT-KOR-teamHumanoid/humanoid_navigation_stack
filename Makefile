SHELL := /bin/bash
.SHELLFLAGS := -e -o pipefail -c
.ONESHELL:
.DEFAULT_GOAL := help

REPOSITORY_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
WORKSPACE_DIR := $(abspath $(REPOSITORY_DIR)/../..)
ROS_SETUP := /opt/ros/jazzy/setup.bash

PATH_PLANNING_PACKAGES := field_coordinate_adapter humanoid_path_planner
NAVIGATION_PACKAGES := $(PATH_PLANNING_PACKAGES) humanoid_path_follower
COLCON_BUILD_ARGS ?= --symlink-install

.PHONY: help build-path-planning test-path-planning start-path-planning \
	debug-path-planning build-navigation test-navigation \
	start-navigation debug-navigation

help:
	@printf '%s\n' \
		'make build-path-planning  - adapter와 path planner 빌드' \
		'make test-path-planning   - adapter와 path planner 테스트' \
		'make start-path-planning  - adapter와 path planner 실행' \
		'make debug-path-planning  - adapter, path planner, RViz 실행' \
		'make build-navigation     - 전체 navigation stack 빌드' \
		'make test-navigation      - 전체 navigation stack 테스트' \
		'make start-navigation     - adapter, planner, follower 실행' \
		'make debug-navigation     - 전체 navigation stack과 RViz 실행'

build-path-planning:
	test -f "$(ROS_SETUP)"
	source "$(ROS_SETUP)"
	cd "$(WORKSPACE_DIR)"
	colcon build $(COLCON_BUILD_ARGS) \
		--packages-select $(PATH_PLANNING_PACKAGES)

test-path-planning:
	test -f "$(ROS_SETUP)"
	source "$(ROS_SETUP)"
	cd "$(WORKSPACE_DIR)"
	colcon test --packages-select $(PATH_PLANNING_PACKAGES)
	colcon test-result --verbose

start-path-planning:
	bash "$(REPOSITORY_DIR)/scripts/start_path_planning.sh"

debug-path-planning:
	bash "$(REPOSITORY_DIR)/scripts/debug_path_planning.sh"

build-navigation:
	test -f "$(ROS_SETUP)"
	source "$(ROS_SETUP)"
	cd "$(WORKSPACE_DIR)"
	colcon build $(COLCON_BUILD_ARGS) \
		--packages-select $(NAVIGATION_PACKAGES)

test-navigation:
	test -f "$(ROS_SETUP)"
	source "$(ROS_SETUP)"
	cd "$(WORKSPACE_DIR)"
	colcon test --packages-select $(NAVIGATION_PACKAGES)
	colcon test-result --verbose

start-navigation:
	bash "$(REPOSITORY_DIR)/scripts/start_navigation.sh"

debug-navigation:
	bash "$(REPOSITORY_DIR)/scripts/debug_navigation.sh"
