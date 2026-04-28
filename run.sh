#!/bin/bash
set -e

#
# USAGE: ./run.sh [--block N]
#


# Set this to use either the c variant or the CUDA variant
SUBDIR="c63-in-c"
SRC_DIR="$(realpath $(dirname $0))/${SUBDIR}"

if [ ! -d $SRC_DIR ]; then
    echo "Please set SUBDIR variable to c63-in-c or c63-in-cuda in $(realpath $0)"
    exit 1
fi

INPUT_FILE="/opt/Media/foreman.yuv"
OUTPUT_FILE="$(realpath $(dirname $0))/output.c63"
WIDTH="352"
HEIGHT="288"

SERVER_CMD="c63server"
WORKER_CMD="c63worker"
WRITER_CMD="c63writer"
DATE=$(date -u +%Y%m%d-%H%M%S)
RSYNC_ARGS="-rt --exclude=logs/ --exclude=.*"
BUILD_DIR="in5050-codec63-build"
BLOCK=""

echo "Source dir: $SRC_DIR"

mkdir -p logs

declare -a workers
declare server

#
# USAGE: ./run.sh [--block N]
#

trap quit INT

function quit()
{
    echo "Cleaning up"

    ssh ${server} "pkill -u \$(whoami) ${SERVER_CMD}" || true
    ssh ${writer} "pkill -u \$(whoami) ${WRITER_CMD}" || true
    for worker in "${workers[@]}"; do
        ssh ${worker} "pkill -u \$(whoami) ${WORKER_CMD}" || true
    done
    
    echo "Logfiles:"
    ls -lh logs/$DATE-*.log
}

function parse_args()
{
    while [ $# -gt 0 ] ; do
        arg=$1
        shift

        case $arg in
            --clean)
                CLEAN="--clean-first"
                ;;
            -w)
                WIDTH=$1
                shift
                ;;
            -h)
                HEIGHT=$1
                shift
                ;;
            --input)
                INPUT_FILE=$1
                shift
                ;;
            --block)
                BLOCK=$1
                shift
                ;;
        esac
    done
    if [ "$BLOCK" == "1" ]; then
        server="dolphin-01"
        workers=("dolphin-02" "dolphin-03")
        writer="dolphin-04"
    elif [ "$BLOCK" == "2" ]; then
        server="dolphin-05"
        workers=("dolphin-07" "dolphin-08")
        writer="dolphin-06"
    elif [ "$BLOCK" == "3" ]; then
        server="dolphin-10"
        workers=("dolphin-09" "dolphin-11")
        writer="dolphin-12"
    else
        echo "unknown compute block $BLOCK"
        echo "USAGE: ./run.sh --block N [OPTIONS]"
        echo "  Where N is 1, 2 or 3"
        echo "  --block    selects which 4 nodes to run on"
        echo "  --input    input file, passed as argument to server"
        echo "  -w WIDTH   video width, passed as '-w' argument to server"
        echo "  -h HEIGHT  video height, passed as '-h' argument to server"
        echo "  --clean    Run cmake clean before building. (For when you're feeling paranoid)"
        exit 1
    fi

    server_node=$(/opt/DIS/sbin/disinfo get-nodeid -hostname "${server}")
    writer_node=$(/opt/DIS/sbin/disinfo get-nodeid -hostname "${writer}")
    args=""

    for worker in "${workers[@]}"; do
        worker_node=$(/opt/DIS/sbin/disinfo get-nodeid -hostname "${worker}")
        args="${args} -r ${worker_node}"
    done

    server_args="${args} -o ${writer_node} -w ${WIDTH} -h ${HEIGHT} ${INPUT_FILE}"
    worker_args="-s ${server_node} ${args}"
    writer_args="-s ${server_node} ${args} -o ./output.c63"

}

function run()
{
    echo "Server ${server} command: ${SERVER_CMD} ${server_args}"
    echo "Writer ${writer} command: ${WRITER_CMD} ${writer_args}"
    for worker in "${workers[@]}"; do
        echo "Worker ${worker} command: ${WORKER_CMD} ${worker_args}"
    done

    #Launch on all nodes
    echo
    echo "### Running ###"
    echo
    stdbuf -oL -eL ssh ${server} "cd $BUILD_DIR/ && stdbuf -oL -eL ./${SERVER_CMD} ${server_args}" |& tee logs/$DATE-server.log &
    stdbuf -oL -eL ssh ${writer} "cd $BUILD_DIR/ && stdbuf -oL -eL ./${WRITER_CMD} ${writer_args}" |& tee logs/$DATE-writer.log &
    for worker in "${workers[@]}"; do
        stdbuf -oL -eL ssh ${worker} "cd $BUILD_DIR/ && stdbuf -oL -eL ./${WORKER_CMD} ${worker_args}" |& tee logs/$DATE-${worker}.log &
    done

    wait

    echo "Copying back output file from writer node"
    rsync ${RSYNC_ARGS} ${writer}:${BUILD_DIR}/output.c63 ${OUTPUT_FILE}
    ls -lh ${OUTPUT_FILE}


}

function compile() {
    echo "### Compiling on localhost ###"
    
    mkdir -p ${SRC_DIR}build
    cmake -S ${SRC_DIR} -B ${SRC_DIR}/build
    cmake --build ${SRC_DIR}/build ${CLEAN}

    echo "### Copying over binaries to remote hosts ###"
    rsync ${RSYNC_ARGS} ${SRC_DIR}/build/${SERVER_CMD} ${server}:${BUILD_DIR}/
    rsync ${RSYNC_ARGS} ${SRC_DIR}/build/${WRITER_CMD} ${writer}:${BUILD_DIR}/
    for worker in "${workers[@]}"; do
        rsync ${RSYNC_ARGS} ${SRC_DIR}/build/${WORKER_CMD} ${worker}:${BUILD_DIR}/
    done
}

parse_args "$@"

compile
run
echo "Done!"

quit
