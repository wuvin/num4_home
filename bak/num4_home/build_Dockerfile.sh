#!/bin/bash

docker buildx build --platform linux/arm64 --network=host -t num4-wuvin:v0 .
