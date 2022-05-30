#include "qmainwindow.h"
#include <QtWidgets/QApplication>
#include <QMainWindow>
#include "vtkoutputwindow.h"
#include "vtkFileOutputWindow.h"
#include "vtkAutoInit.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

int main(int argc, char *argv[])
{
    vtkSmartPointer<vtkFileOutputWindow> fileOutputWindow = vtkSmartPointer<vtkFileOutputWindow>::New();
    fileOutputWindow->SetFileName("Log.txt");
    vtkOutputWindow::SetInstance(fileOutputWindow);
    QApplication a(argc, argv);
    QMainWindow1 w;
    w.show();
    return a.exec();
}
