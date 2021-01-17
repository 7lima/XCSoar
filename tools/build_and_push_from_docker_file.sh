#!/bin/sh
docker login -u gitlab-ci-token -p $CI_BUILD_TOKEN registry.gitlab.com
docker build -t registry.gitlab.com/roeles/xcsoar/$1 -f Dockerfile.$1 .
docker push registry.gitlab.com/roeles/xcsoar/$1

