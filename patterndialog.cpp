#include "patterndialog.h"
#include "ui_patterndialog.h"
#include "qmessagebox.h"

extern bool isICPCustomizeEnable;
extern bool confident;
extern bool isViewPolyEnable1;
extern bool isViewPolyEnable2;
extern bool isViewPolyEnable3;

extern int F_MaxIterations;
extern int F_dotattr;
extern float F_DistanceThreshold;
extern int F_meank;
extern float F_multhresh;
extern float F_filmin;
extern float F_filmax;
extern float F_VoxelX;
extern float F_VoxelY;
extern float F_VoxelZ;
extern float F_RadiusSearch;
extern int F_MinNeighborsInRadius;
extern float F_UsRadius;

extern float R4_ApproxOverlap;
extern float R4_Delta;
extern int R4_MaxComputationTime;
extern int R4_NumberOfSamples;
extern float RI_MaxCorrespondenceDistance;
extern long RI_TransformationEpsilon;
extern float RI_EuclideanFitnessEpsilon;
extern int RI_MaximumIterations;

extern int RC_KS;
extern float RC_SearchRadius;
extern float RC_Mu;
extern int RC_MaximumNearestNeighbors;
extern short RC_MaximumSurfaceAngle;
extern short RC_MinimumAngle;
extern short RC_MaximumAngle;
extern int RC_IsoLevel;
extern int RC_GridResolution;
extern float RC_PercentageExtendGrid;

extern int RC_IsoDivide;

extern float c_Maxdist;
extern float m2p_leafsize;
extern int c_noofSamples;
extern bool isCalAll;

extern int rotatex;

extern float trans1;
extern float trans2;
extern float trans3;

PatternDialog::PatternDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PatternDialog)
{
    ui->setupUi(this);
    QPalette pe;
    pe.setColor(QPalette::WindowText, Qt::red);
    ui->label_28->setPalette(pe);

    //滤波
    QString mi1 = QString::number(F_MaxIterations);
    ui->liIteration->setText(mi1);
    QString mi2 = QString::number(F_dotattr);
    ui->liMass->setText(mi2);
    QString mi3 = QString::number(F_DistanceThreshold);
    ui->liDist->setText(mi3);
    QString mr1 = QString::number(F_meank);
    ui->limeank->setText(mr1);
    QString mr2 = QString::number(F_multhresh);
    ui->liMulthresh->setText(mr2);
    QString mf1 = QString::number(F_filmin);
    ui->liFilMin->setText(mf1);
    QString mf2 = QString::number(F_filmax);
    ui->liFilMax->setText(mf2);
    QString mv1 = QString::number(F_VoxelX);
    ui->liFleafX->setText(mv1);
    QString mv2 = QString::number(F_VoxelY);
    ui->liFleafY->setText(mv2);
    QString mv3 = QString::number(F_VoxelZ);
    ui->liFleafZ->setText(mv3);
    QString mrr1 = QString::number(F_RadiusSearch);
    ui->liradius->setText(mrr1);
    QString mrr2 = QString::number(F_MinNeighborsInRadius);
    ui->liminneibors->setText(mrr2);
    QString mu = QString::number(F_UsRadius);
    ui->liFusRadius->setText(mu);

    //配准
    QString R01 = QString::number(RI_MaxCorrespondenceDistance);
    ui->liR_MaxCorrespondenceDistance->setText(R01);
    QString R02 = QString::number(RI_TransformationEpsilon);
    ui->liR_TransformationEpsilon->setText(R02);
    QString R03 = QString::number(RI_MaximumIterations);
    ui->liR_MaximumIterations->setText(R03);
    QString R04 = QString::number(RI_EuclideanFitnessEpsilon);
    ui->liR_EuclideanFitnessEpsilon->setText(R04);
    QString R05 = QString::number(R4_ApproxOverlap);
    ui->liR_ApproxOverlap->setText(R05);
    QString R06 = QString::number(R4_Delta);
    ui->liR_Delta->setText(R06);
    QString R07 = QString::number(R4_MaxComputationTime);
    ui->liR_MaxComputationTime->setText(R07);
    QString R08 = QString::number(R4_NumberOfSamples);
    ui->liR_NumberOfSamples->setText(R08);
    ui->ICPCustomizeEnable->setChecked(isICPCustomizeEnable);

    //重建 11个变量
    QString RC1 = QString::number(RC_KS);
    ui->liRC_KS->setText(RC1);
    QString RC2 = QString::number(RC_SearchRadius);
    ui->liRC_SearchRadius->setText(RC2);
    QString RC3 = QString::number(RC_Mu);
    ui->liRC_Mu->setText(RC3);
    QString RC4 = QString::number(RC_MaximumNearestNeighbors);
    ui->liRC_MaximumNearestNeighbors->setText(RC4);
    QString RC5 = QString::number(RC_MaximumSurfaceAngle);
    ui->liRC_MaximumSurfaceAngle->setText(RC5);
    QString RC6 = QString::number(RC_MinimumAngle);
    ui->liRC_MinimumAngle->setText(RC6);
    QString RC7 = QString::number(RC_MaximumAngle);
    ui->liRC_MaximumAngle->setText(RC7);
    QString RC8 = QString::number(RC_IsoLevel);
    ui->liRC_IsoLevel->setText(RC8);
    QString RC9 = QString::number(RC_GridResolution);
    ui->liRC_GridResolution->setText(RC9);
    QString RCA = QString::number(RC_PercentageExtendGrid);
    ui->liRC_PercentageExtendGrid->setText(RCA);
    QString RCB = QString::number(RC_IsoDivide);
    ui->liRC_IsoDivide->setText(RCB);
    ui->PreviewPly->setChecked(isViewPolyEnable1);
    ui->SetConfidence1->setChecked(confident);

    //计算
    QString CC1 = QString::number(c_Maxdist);
    ui->liC_Maxdist->setText(CC1);
    QString CC2 = QString::number(m2p_leafsize);
    ui->liC_leafsize->setText(CC2);
    QString NOS = QString::number(c_noofSamples);
    ui->liC_NoOfSamples->setText(NOS);
    ui->c_useall->setChecked(isCalAll);

    //杂项
    QString VTK1 = QString::number(rotatex);
    ui->vtkRotateX->setText(VTK1);
    QString VTK2 = QString::number(trans1);
    ui->vtkicpX->setText(VTK2);
    QString VTK3 = QString::number(trans2);
    ui->vtkicpY->setText(VTK3);
    QString VTK4 = QString::number(trans3);
    ui->vtkicpZ->setText(VTK4);
}

PatternDialog::~PatternDialog()
{
    delete ui;
}

void PatternDialog::on_ApplyMisc_clicked()
{
    //杂项
    int VTK1 = ui->vtkRotateX->text().toInt();
    float VTK2 = ui->vtkicpX->text().toFloat();
    float VTK3 = ui->vtkicpY->text().toFloat();
    float VTK4 = ui->vtkicpZ->text().toFloat();
    rotatex = VTK1;
    trans1 = VTK2;
    trans2 = VTK3;
    trans3 = VTK4;
}

void PatternDialog::on_ApplyM_clicked()
{
    //重建 11个变量
    int RC01 = ui->liRC_KS->text().toInt();
    float RC02 = ui->liRC_SearchRadius->text().toFloat();
    float RC03 = ui->liRC_Mu->text().toFloat();
    int RC04 = ui->liRC_MaximumNearestNeighbors->text().toInt();
    short RC05 = ui->liRC_MaximumSurfaceAngle->text().toShort();
    short RC06 = ui->liRC_MinimumAngle->text().toShort();
    short RC07 = ui->liRC_MaximumAngle->text().toShort();
    int RC08 = ui->liRC_IsoLevel->text().toInt();
    int RC09 = ui->liRC_GridResolution->text().toInt();
    float RC10 = ui->liRC_PercentageExtendGrid->text().toFloat();
    int RC11 = ui->liRC_IsoDivide->text().toInt();
    RC_KS = RC01;
    RC_SearchRadius = RC02;
    RC_Mu = RC03;
    RC_MaximumNearestNeighbors = RC04;
    RC_MaximumSurfaceAngle = RC05;
    RC_MinimumAngle = RC06;
    RC_MaximumAngle = RC07;
    RC_IsoLevel = RC08;
    RC_GridResolution = RC09;
    RC_PercentageExtendGrid = RC10;
    RC_IsoDivide = RC11;
    confident = ui->SetConfidence1->isChecked();
    isViewPolyEnable1 = ui->PreviewPly->isChecked();
    isViewPolyEnable2 = ui->PreviewPly->isChecked();
    isViewPolyEnable3 = ui->PreviewPly->isChecked();
}

void PatternDialog::on_APplyC_clicked()
{
    float CC01 = ui->liC_Maxdist->text().toFloat();
    float CC02 = ui->liC_leafsize->text().toFloat();
    int CC03 = ui->liC_NoOfSamples->text().toInt();
    c_Maxdist = CC01;
    m2p_leafsize = CC02;
    c_noofSamples = CC03;
    isCalAll = ui->c_useall->isChecked();
}

void PatternDialog::on_ApplyR_clicked()
{
    //配准
    float RR01 = ui->liR_ApproxOverlap->text().toFloat();
    float RR02 = ui->liR_Delta->text().toFloat();
    int RR03 = ui->liR_MaxComputationTime->text().toInt();
    int RR04 = ui->liR_NumberOfSamples->text().toInt();
    float RR05 = ui->liR_MaxCorrespondenceDistance->text().toFloat();
    long RR06 = ui->liR_EuclideanFitnessEpsilon->text().toLong();
    float RR07 = ui->liR_EuclideanFitnessEpsilon->text().toFloat();
    int RR08 = ui->liR_MaximumIterations->text().toInt();
    R4_ApproxOverlap = RR01;
    R4_Delta = RR02;
    R4_MaxComputationTime = RR03;
    R4_NumberOfSamples = RR04;
    RI_MaxCorrespondenceDistance = RR05;
    RI_TransformationEpsilon = RR06;
    RI_EuclideanFitnessEpsilon = RR07;
    RI_MaximumIterations = RR08;
    isICPCustomizeEnable = ui->ICPCustomizeEnable->isChecked();
}

void PatternDialog::on_Apply01_clicked()
{
    //滤波
    int i1 = ui->liIteration->text().toInt();
    int i2 = ui->liMass->text().toInt();
    float dt = ui->liDist->text().toFloat();
    int ir1 = ui->limeank->text().toInt();
    float ir2 = ui->liMulthresh->text().toFloat();
    float if1 = ui->liFilMin->text().toFloat();
    float if2 = ui->liFilMax->text().toFloat();
    F_MaxIterations = i1;
    F_dotattr = i2;
    F_DistanceThreshold = dt;
    F_meank = ir1;
    F_multhresh = ir2;
    F_filmin = if1;
    F_filmax = if2;
    float iv1 = ui->liFleafX->text().toFloat();
    float iv2 = ui->liFleafY->text().toFloat();
    float iv3 = ui->liFleafZ->text().toFloat();
    F_VoxelX = iv1;
    F_VoxelY = iv2;
    F_VoxelZ = iv3;
    float irr1 = ui->liradius->text().toFloat();
    F_RadiusSearch = irr1;
    int irr2 = ui->liminneibors->text().toInt();
    F_MinNeighborsInRadius = irr2;
    float iu = ui->liFusRadius->text().toFloat();
    F_UsRadius = iu;
}

void PatternDialog::on_AboutQt_clicked()
{
    QMessageBox::aboutQt(this, "About Qt");
}
