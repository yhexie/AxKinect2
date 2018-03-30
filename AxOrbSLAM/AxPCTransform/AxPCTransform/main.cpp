#include "AxPCTransform.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    AxPCTransform w;
    w.show();
    return a.exec();
}
