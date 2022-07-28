#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ktkim/Workspaces/GNC_wamv/src/navigation"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ktkim/Workspaces/GNC_wamv/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ktkim/Workspaces/GNC_wamv/install/lib/python2.7/dist-packages:/home/ktkim/Workspaces/GNC_wamv/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ktkim/Workspaces/GNC_wamv/build" \
    "/usr/bin/python2" \
    "/home/ktkim/Workspaces/GNC_wamv/src/navigation/setup.py" \
     \
    build --build-base "/home/ktkim/Workspaces/GNC_wamv/build/navigation" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ktkim/Workspaces/GNC_wamv/install" --install-scripts="/home/ktkim/Workspaces/GNC_wamv/install/bin"
