#pragma once

// Override option to prevent unable displaying Chinese in Windows executables
#if defined(_MSC_VER) && (_MSC_VER >= 1600)
# pragma execution_character_set("utf-8")
# endif

#include <QtWidgets/QMainWindow>
#include "ui_qmainwindow.h"
#include "qlabel.h"
#include "patterndialog.h"
// VTK includes
#include <fstream>
#include <sstream>
#include <string>
#include "vtkSmartPointer.h"
#include "vtkSimplePointsReader.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkoutputwindow.h"
#include "vtkVersion.h"
#include "vtkSmartPointer.h"
#include "vtkPoints.h"
#include "vtkVertexGlyphFilter.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "qplaintextedit.h"
#include "qstring.h"
#include "vtkOBJReader.h"
#include "vtkPLYReader.h"
// Pcl includes
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h> 
#include <pcl/registration/icp.h>

class QMainWindow1 : public QMainWindow
{
    Q_OBJECT

public:
    QMainWindow1(QWidget *parent = Q_NULLPTR);

private:
    Ui::QMainWindowClass ui;
    PatternDialog* dialog;
    QString pcFileName = nullptr;
    QString pcFileNameS = nullptr;
    QLabel* label;
    void VTKViewPly(std::string fileName);
    void InitviaICP();
    bool ReadPcd(std::string fileName);
    bool ReadPcd2(std::string fileName);
    bool ReadPly(std::string fileName);
    bool ReadPly2(std::string fileName);
    void PrintToLog(QString* src);
    void UpdateCloud();
    void Txt2Pcd();
    void WritePcd(std::string fileName);
    void WritePly(std::string fileName);
    void PointTempUIUpdater();

private slots:
    void on_OpenFile_clicked();
    void on_DataPreview_clicked();
    void on_butLog_clicked();
    void on_ButICPCal_clicked();
    void on_ButRemodel_clicked();
    void on_actionOpen_triggered();
    void on_action_ToPcd_triggered();
    void on_OpenFile_Sub_clicked();
    void on_butF_ClearInfinity_clicked();
    void on_butF_OverwriteUI_clicked();
    void on_butF_Passthrough_clicked();
    void on_ButViewPortWindow_clicked();
    void on_Preview2Files_clicked();
    void on_butF_RANSAC_clicked();
    void on_butAdvancedSettings_clicked();
    void on_butF_StatF_clicked();
    void on_butF_Voxel_clicked();
    void on_butF_RadiusRemoval_clicked();
    void on_butF_UniSampling_clicked();
    void on_butR_4PCS_clicked();
    void on_butR_ICP_clicked();
    void on_butR_greedy_clicked();
    void on_butR_Cuvism_clicked();
    void on_butR_Poisson_clicked();
    void on_butR_SaveFile_clicked();
    void on_butR_SaveFile_2_clicked();
    void on_butR_SaveFile_3_clicked();
    void on_butR_3DPreview_clicked();
    void on_SaveFIlePCD_clicked();
    void on_action_Save_triggered();
    void on_HideAxis3_stateChanged(int arg1);
    void on_butC_Density_clicked();
    void on_butR_mesh2pcd_clicked();
    void on_butC_21_clicked();
    void on_SetbgColor3_clicked();
    void on_SetMeshColor3_clicked();
    void on_HideAxisW_stateChanged(int arg1);
    void on_HideAxisV_stateChanged(int arg1);
    void on_SetbgColorV_clicked();
    void on_SetPointColorV_clicked();
    void on_SetbgColorW_clicked();
    void on_SetPointColorW1_clicked();
    void on_SetPointColorW2_clicked();
    void on_SetApplyPointSizeW_clicked();
    void on_VTKWApply_clicked();
    void on_butF_OverwriteUI_R_clicked();
    void on_SetPointSizerp_clicked();
    void on_CloseFile_clicked();
    void on_CloseFile_Sub_clicked();
    void on_CloseFile_All_clicked();
    void on_action_Save_2_triggered();
};