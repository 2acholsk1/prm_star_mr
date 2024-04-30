# syntax=docker/dockerfile:1
FROM osrf/ros2:humble

RUN apt-get update -y -q

WORKDIR /app
