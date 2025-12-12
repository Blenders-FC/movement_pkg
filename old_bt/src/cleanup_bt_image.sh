#!/bin/bash

DOT_FILE=$1
DELETE=$2
PNG_FILE="${DOT_FILE%.*}.png"

cleanup() {
  if [ "$DELETE" == "true" ]; then
    echo "[cleanup] Deleting files: $DOT_FILE and $PNG_FILE"
    rm -f "$DOT_FILE" "$PNG_FILE"
  else
    echo "[cleanup] delete_on_exit is false. Skipping cleanup."
  fi
  exit 0
}

# Trap shutdown signals
trap cleanup SIGINT SIGTERM

# Wait forever (until roslaunch stops it)
sleep infinity
