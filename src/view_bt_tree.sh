#!/bin/bash

DOT_FILE=$1
PNG_FILE="${DOT_FILE%.*}.png"

# Wait for the .dot file to be created
echo "[viewer] Waiting for $DOT_FILE..."
while [ ! -f "$DOT_FILE" ]; do
    sleep 0.1
done

echo "[viewer] Found $DOT_FILE. Converting to PNG..."

# Convert to PNG
dot -Tpng "$DOT_FILE" -o "$PNG_FILE"

# Open image
xdg-open "$PNG_FILE"

# Keep the process alive so launch handles shutdown
sleep infinity
