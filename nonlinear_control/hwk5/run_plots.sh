#!/bin/sh

# Create a fancy little progress bar
# https://stackoverflow.com/questions/238073/how-to-add-a-progress-bar-to-a-shell-script
function ProgressBar
{
    # Variables
    s=$(basename ${3})
    i=1

    # Process data
    let _progress=(${1}*100/${2}*100)/100
    let _done=(${_progress}*4)/10
    let _left=40-$_done

    # Build progressbar string lengths
    _fill=$(printf "%${_done}s")
    _empty=$(printf "%${_left}s")

    # 1.2 Build progressbar strings and print the ProgressBar line
    # 1.2.1 Output example:
    # 1.2.1.1 Progress : [########################################] 100%
    printf "\rProcessing $s: [${_fill// /#}${_empty// /-}] ${_progress}%% \r"
}

# Create list of scripts to be ran
declare -a scripts=(`find . -type f -name "*.py" ! -name "plot.py"`)

# Loop through each script
for s in "${scripts[@]}"
do
    ## Update progress bar
    ProgressBar $i ${#scripts[@]} $s
    
    ## Process script
    python $s
    
    ## Update Index
    i=$((i+1))
done

printf '\nFinished!\n'
