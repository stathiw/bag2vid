
#include <QApplication>

#include <ros/ros.h>

#include "bag2vid/frontend/Visualiser.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    bag2vid::Visualiser visualiser;
    visualiser.show();

    return app.exec();
}
