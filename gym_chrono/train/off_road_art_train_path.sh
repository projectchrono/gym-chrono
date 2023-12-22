#!/bin/bash

folder_path="/sbel/Documents/gym-chrono/gym_chrono/train/art_ppo_checkpoints_path"
prefix="ppo_checkpoint"
suffix=".zip"

for ((i=0;i<100;i++)); do
    # Change to the folder directory
    cd "$folder_path" || exit

    # Get the file with the largest number extension
    latest_file=$(ls -1v $prefix*$suffix | tail -n 1)

    if [[ $latest_file =~ $prefix([0-9]+)$suffix ]]; then
        save_point=${BASH_REMATCH[1]}
    fi

    cd ../ || exit
    python off_road_art_train_path.py $save_point
done