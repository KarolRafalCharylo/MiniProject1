#! /bin/bash
shopt -s nullglob

dest_path=~/catkin_ws/src/hello_ros/scripts
if [[ $1 != '' ]];
then
    dest_path=$1
fi

if [[ ! -d "$dest_path" ]]
then
    echo "$dest_path does not exist on your filesystem."
    exit 1
fi

declare -A FILELIST

for f in *.py;
do
    cp $f $dest_path
    chmod +x ${dest_path}/${f}
    FILELIST[$f]=$(
        md5=($(md5sum $f))
        echo $md5
    )
done

trap 'echo "signal caught, cleaning..."; exit 0' SIGINT SIGTERM

while sleep 1; do
    # echo run

    for f in *.py;
    do
        if [[ ! -v FILELIST[$f] ]]; then
            echo "Found new .py: $f"

            cp $f $dest_path
            chmod +x ${dest_path}/${f}
            FILELIST[$f]=$(
                md5=($(md5sum $f))
                echo $md5
            )
        fi
    done

    for key in "${!FILELIST[@]}"; do
        if [[ ${FILELIST[$key]} != $(
                md5=($(md5sum $key))
                echo $md5
        ) ]];
        then
            echo Hash changed for $key
            cp $key $dest_path
            chmod +x ${dest_path}/${key}
            FILELIST[$key]=$(
                md5=($(md5sum $key))
                echo $md5
            )
        fi
    done
done

