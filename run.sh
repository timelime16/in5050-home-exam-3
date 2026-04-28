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


SERVER_ARGS="/opt/Media/foreman.yuv -o output -w 352 -h 288"

SERVER_CMD="c63server"
WORKER_CMD="c63worker"
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
                CLEAN="clean"
                ;;
            --args)
                ARGS=$1
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
        workers=("dolphin-02" "dolphin-03" "dolphin-04")
    elif [ "$BLOCK" == "2" ]; then
        server="dolphin-05"
        workers=("dolphin-06" "dolphin-07" "dolphin-08")
    elif [ "$BLOCK" == "3" ]; then
        server="dolphin-09"
        workers=("dolphin-10" "dolphin-11" "dolphin-12")
    else
        echo "unknown compute block $BLOCK"
        exit 1
    fi

    server_node=$(/opt/DIS/sbin/disinfo get-nodeid -hostname "${server}")
    args=""

    for worker in "${workers[@]}"; do
        worker_node=$(/opt/DIS/sbin/disinfo get-nodeid -hostname "${worker}")
        args="${args} -r ${worker_node}"
    done

    server_args="${SERVER_ARGS} ${args}"
    worker_args=" -s ${server_node} ${args}"

}

function run()
{
    echo "Server command: ${SERVER_CMD} ${server_args}"
    echo "Worker command: ${WORKER_CMD} ${worker_args}"

    #Launch on all nodes
    echo
    echo "### Running ###"
    echo
    stdbuf -oL -eL ssh ${server} "cd $BUILD_DIR/ && stdbuf -oL -eL ./${SERVER_CMD} ${server_args}" |& tee logs/$DATE-server.log &
    for worker in "${workers[@]}"; do
        stdbuf -oL -eL ssh ${worker} "cd $BUILD_DIR/ && stdbuf -oL -eL ./${WORKER_CMD} ${worker_args}" |& tee logs/$DATE-${worker}.log &
    done

    wait

}

function compile() {
    echo "### Compiling on localhost ###"
    
    mkdir -p ${SRC_DIR}build
    cmake -S ${SRC_DIR} -B ${SRC_DIR}/build
    cmake --build ${SRC_DIR}/build

    echo "### Copying over binaries to remote hosts ###"
    rsync ${RSYNC_ARGS} ${SRC_DIR}/build/${SERVER_CMD} ${server}:${BUILD_DIR}/
    for worker in "${workers[@]}"; do
        rsync ${RSYNC_ARGS} ${SRC_DIR}/build/${WORKER_CMD} ${worker}:${BUILD_DIR}/
    done

    #mkdir -p ${BUILD_DIR}/$SUBDIR/build && cd $BUILD_DIR/$SUBDIR/build && cmake .. && make ${CLEAN} $TEGRA_CMD" || exit $?
    #ssh ${server} "cd ${BUILD_DIR}/ && make ${CLEAN} ${SERVER_CMD}"
    #echo "Syncing source"
    #rsync ${RSYNC_ARGS} ${SRC_DIR}/ $TEGRA:${BUILD_DIR}/
    #rsync ${RSYNC_ARGS} ${SRC_DIR}/ $PC:${BUILD_DIR}/
    #echo "Syncing source"
    #rsync ${RSYNC_ARGS} ${SRC_DIR}/ $TEGRA:${BUILD_DIR}/
    #rsync ${RSYNC_ARGS} ${SRC_DIR}/ $PC:${BUILD_DIR}/

    #Compile on tegra and pc
    #echo
    #echo "### Compiling on Tegra ###"
    #ssh -t $TEGRA "cd $BUILD_DIR/tegra-build && make ${CLEAN} $TEGRA_CMD" || exit $?

    #echo
    #echo "### Compiling on PC ###"
    #echo
    #ssh -t $PC "cd $BUILD_DIR/x86-build && make ${CLEAN} $PC_CMD" || exit $?
}

parse_args "$@"

compile
run
echo "Done!"

quit
