#include "qmainwindow.h"
#include "qstringlist.h"
#include "qfileinfo.h"
#include "qmessagebox.h"
#include "qdir.h"
#include "qcolor.h"
#include "qfiledialog.h"
#include "qfile.h"
#include "qtextstream.h"
#include "qprocess.h"
#include "qcolordialog.h"
#include "qpalette.h"
#include "vtkPolyDataReader.h"
#include "vtkSurfaceReconstructionFilter.h"
#include "vtkCamera.h"
#include "vtkContourFilter.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkIterativeClosestPointTransform.h"
#include "vtkLandmarkTransform.h"
#include "vtkAxesActor.h"
#include "vtkCornerAnnotation.h"
#include "vtklight.h"

#include <time.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h> 
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/utils.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>

vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
vtkSmartPointer<vtkPoints> pointsS = vtkSmartPointer<vtkPoints>::New();
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PolygonMesh PM;
pcl::PolygonMesh triangles;

bool lsc = 0;
int pointcount = 0;
int pointcountS = 0;
QString printer = "";
QString ReconstTitle = "";
QString ReconstPrev = "";

//bugfix global attributes
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> fpcs1;
pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ>::Ptr sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr treef(new pcl::KdTreeFLANN<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n11;
pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
pcl::Poisson<pcl::PointNormal> pn;
pcl::search::KdTree <pcl::PointXYZ> trees;
pcl::search::KdTree <pcl::PointXYZ> trees2;
bool isInitialized = 0;


//External Global Attributes
bool isICPCustomizeEnable = 0;
bool confident = 0;
bool isViewPolyEnable1 = 0;
bool isViewPolyEnable2 = 0;
bool isViewPolyEnable3 = 0;
bool Axis3 = 0;
bool AxisW = 0;
bool AxisV = 0;
int F_MaxIterations = 100;
int F_dotattr = 1000;
float F_DistanceThreshold = 0.10;
int F_meank = 50;
float F_multhresh = 1.00;
float F_filmin = 0.00;
float F_filmax = 1.00;
float F_VoxelX = 0.008;
float F_VoxelY = 0.01;
float F_VoxelZ = 0.01;
float F_RadiusSearch = 0.40;
int F_MinNeighborsInRadius = 2;
float F_UsRadius = 0.005;

float R4_ApproxOverlap = 0.7;
float R4_Delta = 0.1;
int R4_MaxComputationTime = 3;
int R4_NumberOfSamples = 1000;
float RI_MaxCorrespondenceDistance = 0.05;
long RI_TransformationEpsilon = 1e8;
float RI_EuclideanFitnessEpsilon = 1.0;
int RI_MaximumIterations = 50;

int RC_KS = 20;
float RC_SearchRadius = 20.0; //设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径【默认值 0】
float RC_Mu = 5.0; //设置最近邻距离的乘子，以得到每个点的最终搜索半径【默认值 0】
int RC_MaximumNearestNeighbors = 150; //设置搜索的最近邻点的最大数量
short RC_MaximumSurfaceAngle = 45; // 90 degrees（pi）最大平面角
short RC_MinimumAngle = 10; // 10 degrees 每个三角的最小角度
short RC_MaximumAngle = 120; // 135 degrees 每个三角的最大角度

int RC_IsoLevel = 0;
int RC_GridResolution = 30;
float RC_PercentageExtendGrid = 0.3;

int RC_IsoDivide = 7;

float c_Maxdist = 0.2;
float m2p_leafsize = 0.1;
int c_noofSamples = 1000;
bool isCalAll = 0;

int rotatex = 30;

float trans1 = 0.2;
float trans2 = 0.0;
float trans3 = 0.0;

//Color attributes
int rb3 = 25;
int gb3 = 25;
int bb3 = 25;
int rb32 = 255;
int gb32 = 255;
int bb32 = 255;
int acdr1 = 255;
int acdg1 = 255;
int acdb1 = 255;
int acdr2 = 255;
int acdg2 = 255;
int acdb2 = 255;
int ps1 = 1;
int ps2 = 1;
int acbr = 25;
int acbg = 25;
int acbb = 25;
int vtkbr = 25;
int vtkbg = 25;
int vtkbb = 25;
int vtkdr = 255;
int vtkdg = 255;
int vtkdb = 255;
int psv = 1;
int psvr = 1;
int regrp = 1;

double getMeanPointDensity(const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float max_dist, int nr_threads)
{
	const float max_dist_sqr = max_dist * max_dist;
	const std::size_t s = cloud->points.size();


	trees.setInputCloud(cloud);

	float mean_dist = 0.f;
	int num = 0;
	std::vector <int> ids(2);
	std::vector <float> dists_sqr(2);
	//--------------多线程加速开始-------------
	pcl::utils::ignore(nr_threads);
#pragma omp parallel for \
  default(none) \
  shared(tree, cloud) \
  firstprivate(ids, dists_sqr) \
  reduction(+:mean_dist, num) \
  firstprivate(s, max_dist_sqr) \
  num_threads(nr_threads)
	//--------------多线程加速结束--------------
	for (int i = 0; i < c_noofSamples; i++)
	{
		trees.nearestKSearch((*cloud)[rand() % s], 2, ids, dists_sqr);
		if (dists_sqr[1] < max_dist_sqr)
		{
			mean_dist += std::sqrt(dists_sqr[1]);
			num++;
		}
	}
	return (mean_dist / num);
};

double compute_cloud_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	trees2.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{    //检查是否存在无效点
		if (!pcl::isFinite(cloud->points[i]))
			continue;

		//Considering the second neighbor since the first is the point itself.
		//在同一个点云内进行k近邻搜索时，k=1的点为查询点本身。
		nres = trees2.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

float caculateRMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target)
{

	float rmse = 0.0f;

	treef->setInputCloud(cloud_target);

	for (auto point_i : *cloud_source)
	{
		// 去除无效的点
		if (!std::isfinite(point_i.x) || !std::isfinite(point_i.y) || !std::isfinite(point_i.z))
			continue;
		pcl::Indices nn_indices(1);
		std::vector<float> nn_distances(1);
		if (!treef->nearestKSearch(point_i, 1, nn_indices, nn_distances)) // K近邻搜索获取匹配点对
			continue;
		size_t point_nn_i = nn_indices.front();
		float dist = squaredEuclideanDistance(point_i, cloud_target->points[point_nn_i]);

		rmse += dist;                 // 计算平方距离之和
	}
	rmse = std::sqrt(rmse / (float) (cloud_source->points.size())); // 计算均方根误差

	return rmse;
}

void QMainWindow1::VTKViewPly(std::string fileName)
{
	QString FileQ = QString::fromStdString(fileName);
	QFileInfo inf(FileQ);
	QString suffix = inf.suffix();
	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();

	vtkSmartPointer<vtkPLYReader> reader2 = vtkSmartPointer<vtkPLYReader>::New();
	vtkPolyDataReader* reader3 = vtkPolyDataReader::New();
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	
	if (suffix == "obj")
	{
		reader->SetFileName(fileName.c_str());
		reader->Update();
		mapper->SetInputConnection(reader->GetOutputPort());
	}
	if (suffix == "ply")
	{
		reader2->SetFileName(fileName.c_str());
		reader2->Update();
		mapper->SetInputConnection(reader2->GetOutputPort());
	}
	if (suffix == "vtk")
	{
		reader3->SetFileName(fileName.c_str());
		reader3->Update();
		mapper->SetInputConnection(reader3->GetOutputPort());
	}
	mapper->ScalarVisibilityOff();
	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(rb32 / 256.0, gb32 / 256.0, bb32 / 256.0);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	QByteArray ba1;
	ba1.append(ReconstPrev);     //也可以 ba2 = s2.toLatin1();
	const char* c1 = ba1.data();

	vtkSmartPointer<vtkCornerAnnotation> t1 = vtkSmartPointer<vtkCornerAnnotation>::New();
	t1->SetText(vtkCornerAnnotation::LowerLeft, c1);
	renderer->AddActor(actor);
	if (!Axis3)
	{
		vtkSmartPointer<vtkAxesActor> ax2 = vtkSmartPointer<vtkAxesActor>::New();
		renderer->AddActor(ax2);
	}

	renderer->AddViewProp(t1);
	renderer->SetBackground(rb3/256.0, gb3 / 256.0, bb3 / 256.0);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(600, 600);
	renderWindow->Render();
	QByteArray ba2;
	ba2.append(ReconstTitle);     //也可以 ba2 = s2.toLatin1();
	const char* c2 = ba2.data();
	renderWindow->SetWindowName(c2);
	renderWindow->Render();

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindow->Render();
	renderWindowInteractor->Start();
}

void QMainWindow1::InitviaICP()
{
	if (!pcFileName.isEmpty() && !pcFileNameS.isEmpty() && !isInitialized)
	{
		if (isInitialized) return;
		QFileInfo inf(pcFileName);
		QString suffix = inf.suffix();
		if (suffix != "pcd" && suffix != "ply") return;
		QFileInfo inf2(pcFileNameS);
		QString suffix2 = inf2.suffix();
		if (suffix2 != "pcd" && suffix2 != "ply") return;
		fpcs1.setInputSource(cloud);
		fpcs1.setInputTarget(cloudS);
		Eigen::Matrix4f trast;
		for (int i = 0; i < 1; i++)
		{
			cloud_temp->clear();
			fpcs1.align(*cloud_temp);
			trast = fpcs1.getFinalTransformation();
		}
		trast = fpcs1.getFinalTransformation();
		pcl::transformPointCloud(*cloud, *cloud_filtered, trast);
		isInitialized = 1;
	}
}

void QMainWindow1::PrintToLog(QString *src)
{
	ui.LogWindow->appendPlainText(*src);
}

void QMainWindow1::UpdateCloud()
{
	if (pointcount != 0)
	{
		points->Reset();
		pointcount = 0;
	}
	for (const auto& point2 : *cloud)
	{
		pointcount = pointcount + 1;
		points->InsertNextPoint(point2.x, point2.y, point2.z);
	}
	ui.TotalDots->setText(QString::asprintf("%d", pointcount));
}

void QMainWindow1::Txt2Pcd() //Txt To Pcd reference
{
	typedef struct tagPOINT_3D
	{
		double x;
		double y;
		double z;
		double r;
	};
	if (pcFileName.isEmpty())
	{
		printer.append(QString::asprintf("没有打开文件"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	QFile pcFile(pcFileName);
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "txt")
	{
		printer.append(QString::asprintf("当前打开的文件不是 txt 格式，不必进行 pcd 格式的转化。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	// 加载txt数据
	int PointNum;
	FILE* fp_txt;
	tagPOINT_3D TxtPoint;
	std::vector<tagPOINT_3D> m_vTxtPoints;
	QByteArray ba0;
	ba0.append(pcFileName);     //也可以 ba2 = s2.toLatin1();
	const char* c2 = ba0.data();
	fp_txt = fopen(ba0, "r");
	uint64_t timestamp = 0;
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
	{
		printer.append(QString::asprintf("转化 txt 文件过程中出错。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	PointNum = m_vTxtPoints.size();
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data  
	cloud->width = PointNum;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = m_vTxtPoints[i].x;
		cloud->points[i].y = m_vTxtPoints[i].y;
		cloud->points[i].z = m_vTxtPoints[i].z;
	}
	QString TgFileName = "";
	TgFileName = QFileDialog::getSaveFileName(this, "保存 Pcd", "", "PCL 点云数据文件 (*.pcd)");
	if (TgFileName.isEmpty()) return;
	ba0.clear();
	ba0.append(TgFileName);     //也可以 ba2 = s2.toLatin1();
	c2 = ba0.data();
	pcl::io::savePCDFileBinary(c2, *cloud);
	printer.append(QString::asprintf("pcd 文件保存成功\n共保存了 %ld 个有效点。", cloud->points.size()));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::PointTempUIUpdater()
{
	long tempDotsTotal = cloud_filtered->width * cloud_filtered->height;
	ui.TotalDots_T->setText(QString::asprintf("%ld", tempDotsTotal));
	ui.TotalDots_rma1->setText(QString::asprintf("%.3f%%", (float)(100.0*(float)tempDotsTotal / pointcount)));
}
//读取pcd文件

bool QMainWindow1::ReadPcd(std::string fileName)
{
	QString Error1 = "";
	if (pointcount != 0)
	{
		points->Reset();
		pointcount = 0;
	}
	cloud->clear();

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloud) == -1) //* load the file
	{
		Error1.append(QString::asprintf("打开 PCD 格式点云文件出错"));
		PrintToLog(&Error1);
		Error1.clear();
		label->setText("打开pcd格式点云文件出错");
		return 0;
	}
	Error1.append(QString::asprintf("已载入 %ld * %ld 个点来自 PCD 文件。", cloud->width, cloud->height));
	PrintToLog(&Error1);
	Error1.clear();
	int c = 1;
	for (const auto& point : *cloud)
	{
		/*Error1.append(QString::asprintf("ID: %5d, %.3f %.3f %.3f", c, point.x, point.y, point.z));
		PrintToLog(&Error1);
		Error1.clear();
		c++;*/
		pointcount = pointcount + 1;
		points->InsertNextPoint(point.x, point.y, point.z);
	}
	return 1;
}
//写入pcd文件
bool QMainWindow1::ReadPcd2(std::string fileName)
{
	QString Error1 = "";
	if (pointcountS != 0)
	{
		points->Reset();
		pointcountS = 0;
	}
	cloudS->clear();

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloudS) == -1) //* load the file
	{
		Error1.append(QString::asprintf("打开 PCD 格式点云文件出错"));
		PrintToLog(&Error1);
		Error1.clear();
		label->setText("打开pcd格式点云文件出错");
		return 0;
	}
	Error1.append(QString::asprintf("已载入 %ld * %ld 个点来自 PCD 文件。", cloudS->width, cloudS->height));
	PrintToLog(&Error1);
	Error1.clear();
	int c = 1;
	for (const auto& point : *cloudS)
	{
		/*Error1.append(QString::asprintf("ID: %5d, %.3f %.3f %.3f", c, point.x, point.y, point.z));
		PrintToLog(&Error1);
		Error1.clear();
		c++;*/
		pointcountS = pointcountS + 1;
		pointsS->InsertNextPoint(point.x, point.y, point.z);
	}
	return 1;
}

bool QMainWindow1::ReadPly(std::string fileName)
{
	QString Error1 = "";
	if (pointcount != 0)
	{
		points->Reset();
		pointcount = 0;
	}
	cloud->clear();

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName, *cloud) == -1) //* load the file
	{
		Error1.append(QString::asprintf("打开 PCD 格式点云文件出错"));
		PrintToLog(&Error1);
		Error1.clear();
		label->setText("打开pcd格式点云文件出错");
		return 0;
	}
	Error1.append(QString::asprintf("已载入 %ld * %ld 个点来自 PLY 文件。", cloud->width, cloud->height));
	PrintToLog(&Error1);
	Error1.clear();
	int c = 1;
	for (const auto& point : *cloud)
	{
		/*Error1.append(QString::asprintf("ID: %5d, %.3f %.3f %.3f", c, point.x, point.y, point.z));
		PrintToLog(&Error1);
		Error1.clear();
		c++;*/
		pointcount = pointcount + 1;
		points->InsertNextPoint(point.x, point.y, point.z);
	}
	return 1;
}

bool QMainWindow1::ReadPly2(std::string fileName)
{
	QString Error1 = "";
	if (pointcountS != 0)
	{
		points->Reset();
		pointcountS = 0;
	}
	cloudS->clear();

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName, *cloudS) == -1) //* load the file
	{
		Error1.append(QString::asprintf("打开 PCD 格式点云文件出错"));
		PrintToLog(&Error1);
		Error1.clear();
		label->setText("打开pcd格式点云文件出错");
		return 0;
	}
	Error1.append(QString::asprintf("已载入 %ld * %ld 个点来自 PLY 文件。", cloudS->width, cloudS->height));
	PrintToLog(&Error1);
	Error1.clear();
	int c = 1;
	for (const auto& point : *cloudS)
	{
		/*Error1.append(QString::asprintf("ID: %5d, %.3f %.3f %.3f", c, point.x, point.y, point.z));
		PrintToLog(&Error1);
		Error1.clear();
		c++;*/
		pointcountS = pointcountS + 1;
		pointsS->InsertNextPoint(point.x, point.y, point.z);
	}
	return 1;
}
//写入pcd文件

void QMainWindow1::WritePcd(std::string fileName)
{
	// Fill in the cloud data

	pcl::io::savePCDFileASCII(fileName, *cloud);
	printer.append(QString::asprintf("已保存 %ld * %ld 个点的 PCD 文件。", cloud->width * cloud->height));
	PrintToLog(&printer);
	printer.clear();

	//for (const auto& point : cloud)
		//cerr << "    " << point.x << " " << point.y << " " << point.z << endl;
	return;
}

void QMainWindow1::WritePly(std::string fileName)
{
	// Fill in the cloud data

	pcl::PLYWriter writer;
	writer.write(fileName, *cloud, true);
	printer.append(QString::asprintf("已保存 %ld * %ld 个点的 PLY 文件。", cloud->width * cloud->height));
	PrintToLog(&printer);
	printer.clear();

	//for (const auto& point : cloud)
		//cerr << "    " << point.x << " " << point.y << " " << point.z << endl;
	return;
}

QMainWindow1::QMainWindow1(QWidget *parent)
    : QMainWindow(parent)
{
	dialog = new PatternDialog(this);
	dialog->setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);
    ui.setupUi(this);
	QColor tempcolor(rb3, gb3, bb3);
    label = new QLabel;
    label->setText(QString::asprintf("就绪"));
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, tempcolor);
	ui.bgc01->setPalette(temppal);
	QColor tempcolor2(rb32, gb32, bb32);
	temppal.setColor(QPalette::WindowText, tempcolor2);
	ui.bgc02->setPalette(temppal);
	QColor tempcolor3(acdr1, acdg1, acdb1);
	temppal.setColor(QPalette::WindowText, tempcolor3);
	ui.bgc04->setPalette(temppal);
	QColor tempcolor4(acdr2, acdg2, acdb2);
	temppal.setColor(QPalette::WindowText, tempcolor4);
	ui.bgc05->setPalette(temppal);
	QColor tempcolor5(acbr, acbg, acbb);
	temppal.setColor(QPalette::WindowText, tempcolor5);
	ui.bgc03->setPalette(temppal);
	QColor tempcolor6(vtkbr, vtkbg, vtkbb);
	temppal.setColor(QPalette::WindowText, tempcolor6);
	ui.VTKBColor->setPalette(temppal);
	QColor tempcolor7(vtkdr, vtkdg, vtkdb);
	temppal.setColor(QPalette::WindowText, tempcolor7);
	ui.VTKDColor->setPalette(temppal);
	QString temp;
	temp = QString::number(ps1);
	ui.pointsz1->setText(temp);
	temp = QString::number(ps2);
	ui.pointsz2->setText(temp);
	temp = QString::number(psv);
	ui.pointszV->setText(temp);
	temp = QString::number(psvr);
	ui.pointszVR->setText(temp);
	temp = QString::number(regrp);
	ui.pointsrp->setText(temp);
}

void QMainWindow1::on_OpenFile_clicked()
{
	QString PCF;
	PCF = QFileDialog::getOpenFileName(this, "打开点云文件", "", "文本文件 (*.txt);;PCL 点云数据文件 (*.pcd);;PLY 点云数据文件 (*.ply);;全部文件 (*)");
	if (PCF.isEmpty()) return;
	pcFileName = PCF;
    if (pcFileName.isEmpty()) return;
    ui.FileDir->setText(pcFileName);
	if (pointcount != 0)
	{
		points->Reset();
		pointcount = 0;
	}
    QFile pcFile(pcFileName);
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix == "txt")
	{
		pcFile.open(QIODevice::ReadOnly | QIODevice::Text);
		std::string filename = pcFileName.toStdString();
		std::ifstream filestream(filename.c_str());
		std::string line;
		double xyz[3] = { 0.0, 0.0, 0.0 };
		while (std::getline(filestream, line))
		{
			std::stringstream linestream;
			linestream << line;
			linestream >> xyz[0] >> xyz[1] >> xyz[2];
			pointcount = pointcount + 1;
			points->InsertNextPoint(xyz[0], xyz[1], xyz[2]);
		}
		ui.FileType->setText(QString::asprintf("Regular Text"));
		filestream.close();
		pcFile.close();
	}
	else if (suffix == "pcd")
	{
		if (!ReadPcd(pcFileName.toStdString()))
		{
			printer.append(QString::asprintf("打开 PCD 格式点云文件出错"));
			PrintToLog(&printer);
			printer.clear();
			label->setText("打开 PCD 格式点云文件出错");
		}
		else ui.FileType->setText(QString::asprintf("Pcd Data"));
	}
	else if (suffix == "ply")
	{
		if (!ReadPly(pcFileName.toStdString()))
		{
			printer.append(QString::asprintf("打开 PLY 格式点云文件出错"));
			PrintToLog(&printer);
			printer.clear();
			label->setText("打开 PLY 格式点云文件出错");
		}
		else ui.FileType->setText(QString::asprintf("Ply Data"));
	}
    ui.DataPreview->setEnabled(1);
	ui.CloseFile->setEnabled(1);
	ui.ButRemodel->setEnabled(1);
	ui.CloseFile_All->setEnabled(1);
    ui.TotalDots->setText(QString::asprintf("%d", pointcount));
}

void QMainWindow1::on_DataPreview_clicked()
{
    vtkSmartPointer<vtkPolyData> polyData =
        vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    glyphFilter->SetInputConnection(polyData->GetProducerPort());
#else
    glyphFilter->SetInputData(polyData);
#endif
    glyphFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
        vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(vtkdr / 255.0, vtkdg / 255.0, vtkdb / 255.0);
	actor->GetProperty()->SetPointSize(psv);
    vtkSmartPointer<vtkRenderer> renderer =
        vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(vtkbr / 255.0, vtkbg / 255.0, vtkbb / 255.0);
	if (!AxisV)
	{
		vtkSmartPointer<vtkAxesActor> axesActor =
			vtkSmartPointer<vtkAxesActor>::New();
		renderer->AddActor(axesActor);
	}
    ui.qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    ui.ButICPCal->setEnabled(1);
    ui.ButRemodel->setEnabled(1);
}

void QMainWindow1::on_butLog_clicked()
{
    QProcess* proc = new QProcess();
    QString path = ("notepad.exe Log.txt");
    QString pathf = QString("Log.txt");
    QFile file(pathf);
    if (!file.exists())
    {
        label->setText(QString::asprintf("找不到日志文件"));
        return;
    }
    proc->start(path);
}

void QMainWindow1::on_ButICPCal_clicked()
{
	//--------------------------------读取数据----------------------------
	vtkSmartPointer<vtkPolyData> polyData =
		vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);
	//---------------------------输入变换矩阵进行变换----------------------
	vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
	translation->Translate(trans1, trans2, trans3);
	translation->RotateX(rotatex);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter1 =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter1->SetInputData(polyData);
	transformFilter1->SetTransform(translation);
	transformFilter1->Update();

	vtkSmartPointer<vtkPolyData> source = vtkSmartPointer<vtkPolyData>::New();
	source->SetPoints(points);
	vtkSmartPointer<vtkPolyData> target = vtkSmartPointer<vtkPolyData>::New();
	target->SetPoints(transformFilter1->GetOutput()->GetPoints());
	//-------------将输入的点集转换为vtkPolyData数据-----------------------
	vtkSmartPointer<vtkVertexGlyphFilter> sourceGlyphFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	sourceGlyphFilter->SetInputData(source);
	sourceGlyphFilter->Update();

	vtkSmartPointer<vtkVertexGlyphFilter> targetGlyphFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	targetGlyphFilter->SetInputData(target);
	targetGlyphFilter->Update();
	//----------------------------ICP配准----------------------------------
	vtkSmartPointer<vtkIterativeClosestPointTransform> icpTransform =
		vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
	icpTransform->SetSource(sourceGlyphFilter->GetOutput());
	icpTransform->SetTarget(targetGlyphFilter->GetOutput());
	icpTransform->GetLandmarkTransform()->SetModeToRigidBody();
	icpTransform->SetMaximumNumberOfIterations(20); //Iterations
	icpTransform->StartByMatchingCentroidsOn();
	icpTransform->Modified();
	icpTransform->Update();

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter2 =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter2->SetInputData(sourceGlyphFilter->GetOutput());
	transformFilter2->SetTransform(icpTransform);
	transformFilter2->Update();
	//--------------------------可视化-----------------------------------
	vtkSmartPointer<vtkPolyDataMapper> sourceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sourceMapper->SetInputConnection(sourceGlyphFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> sourceActor = vtkSmartPointer<vtkActor>::New();
	sourceActor->SetMapper(sourceMapper);
	sourceActor->GetProperty()->SetColor(0, 1, 0);
	sourceActor->GetProperty()->SetPointSize(regrp);
	int countG = sourceActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkPolyDataMapper> targetMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	targetMapper->SetInputConnection(targetGlyphFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> targetActor = vtkSmartPointer<vtkActor>::New();
	targetActor->SetMapper(targetMapper);
	targetActor->GetProperty()->SetColor(1, 0, 0);
	targetActor->GetProperty()->SetPointSize(regrp);
	int countR = targetActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkPolyDataMapper> solutionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	solutionMapper->SetInputConnection(transformFilter2->GetOutputPort());

	vtkSmartPointer<vtkActor> solutionActor = vtkSmartPointer<vtkActor>::New();
	solutionActor->SetMapper(solutionMapper);
	solutionActor->GetProperty()->SetColor(0, 0, 1);
	solutionActor->GetProperty()->SetPointSize(regrp);
	int countB = solutionActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->SetBackground(1.0, 1.0, 1.0);

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderer->AddActor(sourceActor);
	renderer->AddActor(targetActor);
	renderer->AddActor(solutionActor);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->SetSize(600, 600);
	renderWindow->Render();
	renderWindow->SetWindowName("ICP Registration Preview");
	renderWindow->Render();
	renderWindow->Render();
	renderWindowInteractor->Start();
}

void QMainWindow1::on_ButRemodel_clicked()
{
	QMessageBox::StandardButton yes1;
	yes1 = QMessageBox::warning(this, "警告", "该方法不稳定，使用部分模型可能崩溃\n可尝试修改高级设置内的数值，但作用似乎不大。\n确认要继续执行吗？", QMessageBox::Yes | QMessageBox::No);
	if (yes1 != QMessageBox::Yes) return;
	vtkSmartPointer<vtkPolyData> points1 =
		vtkSmartPointer<vtkPolyData>::New();
	points1->SetPoints(points); //获得网格模型中的几何数据：点集

	vtkSmartPointer<vtkSurfaceReconstructionFilter> surf =
		vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
	surf->SetInputData(points1);
	surf->SetNeighborhoodSize(20);
	surf->SetSampleSpacing(0.005);
	surf->Update();

	vtkSmartPointer<vtkContourFilter> contour =
		vtkSmartPointer<vtkContourFilter>::New();
	contour->SetInputConnection(surf->GetOutputPort());
	contour->SetValue(0, 0.0);
	contour->Update();
	//
	vtkSmartPointer <vtkVertexGlyphFilter> vertexGlyphFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexGlyphFilter->AddInputData(points1);
	vertexGlyphFilter->Update();
	vtkSmartPointer<vtkPolyDataMapper> pointMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	pointMapper->SetInputData(vertexGlyphFilter->GetOutput());
	pointMapper->ScalarVisibilityOff();

	vtkSmartPointer<vtkActor> pointActor =
		vtkSmartPointer<vtkActor>::New();
	pointActor->SetMapper(pointMapper);
	pointActor->GetProperty()->SetColor(0, 0, 1);
	pointActor->GetProperty()->SetPointSize(psvr);

	vtkSmartPointer<vtkPolyDataMapper> contourMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	contourMapper->SetInputData(contour->GetOutput());
	vtkSmartPointer<vtkActor> contourActor =
		vtkSmartPointer<vtkActor>::New();
	contourActor->SetMapper(contourMapper);
	///
	double pointView[4] = { 0, 0, 0.5, 1 };
	double contourView[4] = { 0.5, 0, 1, 1 };

	vtkSmartPointer<vtkRenderer> pointRender =
		vtkSmartPointer<vtkRenderer>::New();
	pointRender->AddActor(pointActor);
	pointRender->SetViewport(pointView);
	pointRender->SetBackground(1, 1, 1);

	vtkSmartPointer<vtkRenderer> contourRender =
		vtkSmartPointer<vtkRenderer>::New();
	contourRender->AddActor(contourActor);
	contourRender->SetViewport(contourView);
	contourRender->SetBackground(1, 1, 1);

	pointRender->GetActiveCamera()->SetPosition(0, -1, 0);
	pointRender->GetActiveCamera()->SetFocalPoint(0, 0, 0);
	pointRender->GetActiveCamera()->SetViewUp(0, 0, 1);
	pointRender->GetActiveCamera()->Azimuth(30);
	pointRender->GetActiveCamera()->Elevation(30);
	pointRender->ResetCamera();
	contourRender->SetActiveCamera(pointRender->GetActiveCamera());

	vtkSmartPointer<vtkRenderWindow> rw =
		vtkSmartPointer<vtkRenderWindow>::New();
	rw->AddRenderer(pointRender);
	rw->AddRenderer(contourRender);
	rw->SetSize(1200,600);
	rw->SetWindowName("Remodeling via VTK");
	rw->Render();

	vtkSmartPointer<vtkRenderWindowInteractor> rwi =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	rwi->SetRenderWindow(rw);
	rwi->Initialize();
	rwi->Start();
}

void QMainWindow1::on_actionOpen_triggered()
{
	on_OpenFile_clicked();
}

void QMainWindow1::on_action_ToPcd_triggered()
{
	Txt2Pcd();
}

void QMainWindow1::on_OpenFile_Sub_clicked()
{
	QString PCF;
	PCF = QFileDialog::getOpenFileName(this, "打开点云文件", "", "文本文件 (*.txt);;PCL 点云数据文件 (*.pcd);;PLY 点云数据文件 (*.ply);;全部文件 (*)");
	if (PCF.isEmpty()) return;
	pcFileNameS = PCF;
	if (pcFileNameS.isEmpty()) return;
	ui.FileDir_Sub->setText(pcFileNameS);
	if (pointcountS != 0)
	{
		pointsS->Reset();
		pointcountS = 0;
	}
	QFile pcFile(pcFileNameS);
	QFileInfo inf(pcFileNameS);
	QString suffix = inf.suffix();
	if (suffix == "txt")
	{
		pcFile.open(QIODevice::ReadOnly | QIODevice::Text);
		std::string filename = pcFileNameS.toStdString();
		std::ifstream filestream(filename.c_str());
		std::string line;
		double xyz[3] = { 0.0, 0.0, 0.0 };
		while (std::getline(filestream, line))
		{
			std::stringstream linestream;
			linestream << line;
			linestream >> xyz[0] >> xyz[1] >> xyz[2];
			pointcountS = pointcountS + 1;
			pointsS->InsertNextPoint(xyz[0], xyz[1], xyz[2]);
		}
		ui.FileType_2->setText(QString::asprintf("Regular Text"));
		filestream.close();
		pcFile.close();
	}
	else if (suffix == "pcd")
	{
		if (!ReadPcd2(pcFileNameS.toStdString()))
		{
			printer.append(QString::asprintf("打开 PCD 格式点云文件出错"));
			PrintToLog(&printer);
			printer.clear();
			label->setText("打开 PCD 格式点云文件出错");
		}
		else ui.FileType_2->setText(QString::asprintf("Pcd Data"));
	}
	else if (suffix == "ply")
	{
		if (!ReadPly2(pcFileNameS.toStdString()))
		{
			printer.append(QString::asprintf("打开 PLY 格式点云文件出错"));
			PrintToLog(&printer);
			printer.clear();
			label->setText("打开 PLY 格式点云文件出错");
		}
		else ui.FileType_2->setText(QString::asprintf("Ply Data"));
	}
	ui.CloseFile_Sub->setEnabled(1);
	ui.CloseFile_All->setEnabled(1);
	ui.TotalDots_2->setText(QString::asprintf("%d", pointcountS));
}

void QMainWindow1::on_butF_ClearInfinity_clicked()
{
	InitviaICP();
	ui.LogWindow->clear();
	label->setText(QString::asprintf("正在处理 - 剔除离群点"));
	QString temp = "";

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	if (pcFileName.isEmpty())
	{
		temp.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		temp.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	sor->setInputCloud(cloud);
	// 设置过滤邻域K
	sor->setMeanK(F_meank);
	// 设置标准差的系数, 值越大，丢掉的点越少
	sor->setStddevMulThresh(F_multhresh);
	cloud_filtered->clear();
	sor->filter(*cloud_filtered);
	label->setText(QString::asprintf("处理完成 - 剔除离群点"));

	temp.append(QString::asprintf("--- 利用 StatisticalOutlierRemoval 算法剔除离群点\n参数:\n过滤邻域 K = %d", F_meank));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("标准差倍率 = %.2f", F_multhresh));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("处理前后参数对比: "));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("点云宽度: %ld -> %ld", cloud->width, cloud_filtered->width));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("点云高度: %ld -> %ld", cloud->height, cloud_filtered->height));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("总点数: %ld -> %ld", cloud->height* cloud->width, cloud_filtered->height* cloud_filtered->width));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("剩余采样点占比 = %.3f%%", (float)((float)(cloud_filtered->width * cloud_filtered->height) / (float)(cloud->width * cloud->height)) * 100));
	PrintToLog(&temp);
	temp.clear();
	ui.TempOperationName->setText(QString::asprintf("剔除离群点"));
	PointTempUIUpdater();

	/*pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);*/
}
//ok

void QMainWindow1::on_butF_OverwriteUI_clicked()
{
	if (cloud_filtered->width == 0 || cloud_filtered->height == 0)
	{
		printer.append(QString::asprintf("没有可供保存的处理结果。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	cloud->clear();
	*cloud = *cloud_filtered;
	cloud_filtered->clear();
	UpdateCloud();
	printer.append(QString::asprintf("数据覆盖成功。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_butF_Passthrough_clicked()
{
	InitviaICP();
	ui.LogWindow->clear();
	cloud_filtered->clear();
	if (pcFileName.isEmpty())
	{
		printer.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&printer);
		printer.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		printer.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&printer);
		printer.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}

	printer.append(QString::asprintf("--- 直通滤波 PassThrough\n参数:\n过滤前总点数 = %ld", cloud->width*cloud->height));
	PrintToLog(&printer);
	printer.clear();

	// Create the filtering object
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(F_filmin, F_filmax);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filtered);

	printer.append(QString::asprintf("过滤后总点数 = %ld", cloud_filtered->width * cloud_filtered->height));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("有效范围 = %.3f ~ %.3f", F_filmin, F_filmax));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("剩余采样点占比 = %.3f%%", (float)((float)(cloud_filtered->width * cloud_filtered->height)/(float)(cloud->width * cloud->height))*100));
	PrintToLog(&printer);
	printer.clear();
	ui.TempOperationName->setText(QString::asprintf("直通滤波"));
	PointTempUIUpdater();
}
//ok

void QMainWindow1::on_ButViewPortWindow_clicked()
{
	vtkSmartPointer<vtkPoints> pointsP = vtkSmartPointer<vtkPoints>::New();
	int pointcount2 = 0;
	vtkSmartPointer<vtkPolyData> p1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> p2 = vtkSmartPointer<vtkPolyData>::New();
	for (const auto& point : *cloud_filtered)
	{

		pointcount2 = pointcount2 + 1;
		pointsP->InsertNextPoint(point.x, point.y, point.z);
	}
	p1->SetPoints(points);
	p2->SetPoints(pointsP);
	vtkSmartPointer<vtkVertexGlyphFilter> g1 =vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g1->SetInputData(p1);
	vtkSmartPointer<vtkVertexGlyphFilter> g2 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g2->SetInputData(p2);
	g1->Update();
	g2->Update();
	vtkSmartPointer<vtkRenderWindow> wind = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkPolyDataMapper> m1 = vtkSmartPointer<vtkPolyDataMapper>::New();
	m1->SetInputConnection(g1->GetOutputPort());
	vtkSmartPointer<vtkPolyDataMapper> m2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	m2->SetInputConnection(g2->GetOutputPort());
	vtkSmartPointer<vtkActor> a1 = vtkSmartPointer<vtkActor>::New();
	a1->SetMapper(m1);
	a1->GetProperty()->SetColor(acdr1 / 255.0, acdg1 / 255.0, acdb1 / 255.0);
	a1->GetProperty()->SetPointSize(ps1);
	vtkSmartPointer<vtkActor> a2 = vtkSmartPointer<vtkActor>::New();
	a2->SetMapper(m2);
	a2->GetProperty()->SetColor(acdr2 / 255.0, acdg2 / 255.0, acdb2 / 255.0);
	a2->GetProperty()->SetPointSize(ps2);
	vtkSmartPointer<vtkRenderer> r1 = vtkSmartPointer<vtkRenderer>::New();
	r1->AddActor(a1);
	r1->SetBackground(acbr / 255.0, acbg / 255.0, acbb / 255.0);
	vtkSmartPointer<vtkRenderer> r2 = vtkSmartPointer<vtkRenderer>::New();
	r2->AddActor(a2);
	r2->SetBackground(acbr / 255.0, acbg / 255.0, acbb / 255.0);
	if (!AxisW)
	{
		vtkSmartPointer<vtkAxesActor> ax1 = vtkSmartPointer<vtkAxesActor>::New();
		r1->AddActor(ax1);
		vtkSmartPointer<vtkAxesActor> ax2 = vtkSmartPointer<vtkAxesActor>::New();
		r2->AddActor(ax2);
	}
	QString str1 = QString::asprintf("Original Data: %ld Points", pointcount);
	QString str2 = QString::asprintf("Filtered Data: %ld Points (%.2f%%)", pointcount2, (float)(100.0*((float)pointcount2/(float)pointcount)));
	QByteArray ba1;
	ba1.append(str1);     //也可以 ba2 = s2.toLatin1();
	const char* c1 = ba1.data();
	QByteArray ba2;
	ba2.append(str2);     //也可以 ba2 = s2.toLatin1();
	const char* c2 = ba2.data();

	vtkSmartPointer<vtkCornerAnnotation> t1 = vtkSmartPointer<vtkCornerAnnotation>::New();
	t1->SetText(vtkCornerAnnotation::LowerLeft, c1);
	vtkSmartPointer<vtkCornerAnnotation> t2 = vtkSmartPointer<vtkCornerAnnotation>::New();
	t2->SetText(vtkCornerAnnotation::LowerLeft, c2);
	r1->AddViewProp(t1);
	r2->AddViewProp(t2);
	wind->AddRenderer(r1);
	wind->AddRenderer(r2);
	r1->SetViewport(0.0, 0.0, 0.5, 1.0);
	r2->SetViewport(0.5, 0.0, 1.0, 1.0);
	r1->GetActiveCamera()->SetPosition(0, -1, 0);
	r1->GetActiveCamera()->SetFocalPoint(0, 0, 0);
	r1->GetActiveCamera()->SetViewUp(0, 0, 1);
	r1->GetActiveCamera()->Azimuth(30);
	r1->GetActiveCamera()->Elevation(30);
	r1->ResetCamera();
	r2->SetActiveCamera(r1->GetActiveCamera());
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	wind->SetSize(1200, 600);
	
	renderWindowInteractor->SetRenderWindow(wind);
	renderWindowInteractor->Initialize();
	renderWindowInteractor->Start();
}

void QMainWindow1::on_Preview2Files_clicked()
{
	int pointcount2 = 0;
	vtkSmartPointer<vtkPolyData> p1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> p2 = vtkSmartPointer<vtkPolyData>::New();
	p1->SetPoints(points);
	p2->SetPoints(pointsS);
	vtkSmartPointer<vtkVertexGlyphFilter> g1 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g1->SetInputData(p1);
	vtkSmartPointer<vtkVertexGlyphFilter> g2 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g2->SetInputData(p2);
	g1->Update();
	g2->Update();
	vtkSmartPointer<vtkRenderWindow> wind = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkPolyDataMapper> m1 = vtkSmartPointer<vtkPolyDataMapper>::New();
	m1->SetInputConnection(g1->GetOutputPort());
	vtkSmartPointer<vtkPolyDataMapper> m2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	m2->SetInputConnection(g2->GetOutputPort());
	vtkSmartPointer<vtkActor> a1 = vtkSmartPointer<vtkActor>::New();
	a1->SetMapper(m1);
	a1->GetProperty()->SetColor(acdr1 / 255.0, acdg1 / 255.0, acdb1 / 255.0);
	a1->GetProperty()->SetPointSize(ps1);
	vtkSmartPointer<vtkActor> a2 = vtkSmartPointer<vtkActor>::New();
	a2->SetMapper(m2);
	a2->GetProperty()->SetColor(acdr2 / 255.0, acdg2 / 255.0, acdb2 / 255.0);
	a2->GetProperty()->SetPointSize(ps2);
	vtkSmartPointer<vtkRenderer> r1 = vtkSmartPointer<vtkRenderer>::New();
	r1->AddActor(a1);
	r1->SetBackground(acbr / 255.0, acbg / 255.0, acbb / 255.0);
	vtkSmartPointer<vtkRenderer> r2 = vtkSmartPointer<vtkRenderer>::New();
	r2->AddActor(a2);
	r2->SetBackground(acbr / 255.0, acbg / 255.0, acbb / 255.0);
	if (!AxisW)
	{
		vtkSmartPointer<vtkAxesActor> ax1 = vtkSmartPointer<vtkAxesActor>::New();
		r1->AddActor(ax1);
		vtkSmartPointer<vtkAxesActor> ax2 = vtkSmartPointer<vtkAxesActor>::New();
		r2->AddActor(ax2);
	}
	QString str1 = QString::asprintf("Data 1: %ld Points", pointcount);
	QString str2 = QString::asprintf("Data 2: %ld Points", pointcountS);
	QByteArray ba1;
	ba1.append(str1);     //也可以 ba2 = s2.toLatin1();
	const char* c1 = ba1.data();
	QByteArray ba2;
	ba2.append(str2);     //也可以 ba2 = s2.toLatin1();
	const char* c2 = ba2.data();

	vtkSmartPointer<vtkCornerAnnotation> t1 = vtkSmartPointer<vtkCornerAnnotation>::New();
	t1->SetText(vtkCornerAnnotation::LowerLeft, c1);
	vtkSmartPointer<vtkCornerAnnotation> t2 = vtkSmartPointer<vtkCornerAnnotation>::New();
	t2->SetText(vtkCornerAnnotation::LowerLeft, c2);
	r1->AddViewProp(t1);
	r2->AddViewProp(t2);
	wind->AddRenderer(r1);
	wind->AddRenderer(r2);
	r1->SetViewport(0.0, 0.0, 0.5, 1.0);
	r2->SetViewport(0.5, 0.0, 1.0, 1.0);
	r1->GetActiveCamera()->SetPosition(0, -1, 0);
	r1->GetActiveCamera()->SetFocalPoint(0, 0, 0);
	r1->GetActiveCamera()->SetViewUp(0, 0, 1);
	r1->GetActiveCamera()->Azimuth(30);
	r1->GetActiveCamera()->Elevation(30);
	r1->ResetCamera();
	r2->SetActiveCamera(r1->GetActiveCamera());
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	wind->SetSize(1200, 600);

	renderWindowInteractor->SetRenderWindow(wind);
	renderWindowInteractor->Initialize();
	renderWindowInteractor->Start();
}

void QMainWindow1::on_butF_RANSAC_clicked()
{
	InitviaICP();
	ui.LogWindow->clear();
	if (pointcount == 0)
	{
		printer.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	//由于原有的table点云数据量太大，进行降采样
	cloud_temp->clear();
	cloud_filtered->clear();
	pcl::VoxelGrid<pcl::PointXYZ> vfilter;
	vfilter.setInputCloud(cloud);
	float dat = 1 / (float)F_dotattr;
	vfilter.setLeafSize(dat, dat, dat);
	vfilter.filter(*cloud_temp);

	//
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr indices(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> sacseg;
	sacseg.setOptimizeCoefficients(true);
	sacseg.setModelType(pcl::SACMODEL_PLANE);
	sacseg.setMethodType(pcl::SAC_RANSAC);
	sacseg.setMaxIterations(F_MaxIterations);//最大迭代次数
	sacseg.setDistanceThreshold(F_DistanceThreshold);//距离阈值
	sacseg.setInputCloud(cloud_temp);
	sacseg.segment(*indices, *coefficients);
	printer.append(QString::asprintf("--- RANSAC分割平面"));
	PrintToLog(&printer);
	printer.clear();

	if (indices->indices.size() == 0)
	{
		printer.append(QString::asprintf("无法根据给定数据集估计平面模型。"));
		PrintToLog(&printer);
		printer.clear();
	}
	pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
	extractIndices.setInputCloud(cloud_temp);
	extractIndices.setIndices(indices);
	extractIndices.setNegative(!(ui.checkBox_NNegative->isChecked()));
	extractIndices.filter(*cloud_filtered);
	label->setText(QString::asprintf("处理完成 - 剔除离群点"));

	printer.append(QString::asprintf("最大迭代次数 = %d", F_MaxIterations));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("距离阈值 = %.2f", F_DistanceThreshold));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("点密度系数 = %d", F_dotattr));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("处理前后参数对比: "));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("总点数: %ld -> %ld", cloud->height * cloud->width, cloud_filtered->height * cloud_filtered->width));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("剩余采样点占比 = %.3f%%", (float)((float)(cloud_filtered->width * cloud_filtered->height) / (float)(cloud->width * cloud->height)) * 100));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("模型系数为： \n%.3f %.3f\n%.3f %.3f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]));
	PrintToLog(&printer);
	printer.clear();
	ui.TempOperationName->setText(QString::asprintf("RANSAC 平面分割"));
	PointTempUIUpdater();
}
//ok

void QMainWindow1::on_butAdvancedSettings_clicked()
{
	dialog->show();
}

void QMainWindow1::on_butF_StatF_clicked()
{
	InitviaICP();
	ui.LogWindow->clear();
	label->setText(QString::asprintf("正在处理 - 统计学滤波"));
	QString temp = "";

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	if (pcFileName.isEmpty())
	{
		temp.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		temp.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	
	sor->setInputCloud(cloud);
	// 设置过滤邻域K
	sor->setMeanK(F_meank);
	// 设置标准差的系数, 值越大，丢掉的点越少
	sor->setStddevMulThresh(F_multhresh);
	cloud_filtered->clear();
	sor->filter(*cloud_filtered);
	label->setText(QString::asprintf("处理完成 - 统计学滤波"));

	temp.append(QString::asprintf("--- 统计学滤波\n参数:\n过滤邻域 K = %d", F_meank));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("标准差倍率 = %.2f", F_multhresh));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("处理前后参数对比: "));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("点云宽度: %ld -> %ld", cloud->width, cloud_filtered->width));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("点云高度: %ld -> %ld", cloud->height, cloud_filtered->height));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("总点数: %ld -> %ld", cloud->height * cloud->width, cloud_filtered->height * cloud_filtered->width));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("剩余采样点占比 = %.3f%%", (float)((float)(cloud_filtered->width * cloud_filtered->height) / (float)(cloud->width * cloud->height)) * 100));
	PrintToLog(&temp);
	temp.clear();
	ui.TempOperationName->setText(QString::asprintf("统计学滤波"));
	PointTempUIUpdater();
}
//ok

void QMainWindow1::on_butF_Voxel_clicked()
{
	InitviaICP();
	ui.LogWindow->clear();
	label->setText(QString::asprintf("正在处理 - 体素滤波"));
	QString temp = "";

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	if (pcFileName.isEmpty())
	{
		temp.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		temp.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		return;
	}

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(F_VoxelX, F_VoxelY, F_VoxelZ);
	sor.filter(*cloud_filtered);
	label->setText(QString::asprintf("处理完成 - 体素滤波"));

	temp.append(QString::asprintf("--- 体素滤波\n参数:"));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("叶片大小 = %.2f * %.2f * %.2f", F_VoxelX, F_VoxelY, F_VoxelZ));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("处理前后参数对比: "));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("总点数: %ld -> %ld", cloud->height * cloud->width, cloud_filtered->height * cloud_filtered->width));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("剩余采样点占比 = %.3f%%", (float)((float)(cloud_filtered->width * cloud_filtered->height) / (float)(cloud->width * cloud->height)) * 100));
	PrintToLog(&temp);
	temp.clear();
	ui.TempOperationName->setText(QString::asprintf("体素滤波"));
	PointTempUIUpdater();
}
//ok

void QMainWindow1::on_butF_RadiusRemoval_clicked()
{
	ui.LogWindow->clear();
	label->setText(QString::asprintf("正在处理 - 半径滤波"));
	QString temp = "";

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	if (pcFileName.isEmpty())
	{
		temp.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		temp.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}

	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(F_RadiusSearch);
	outrem.setMinNeighborsInRadius(F_MinNeighborsInRadius);
	outrem.filter(*cloud_filtered);
	label->setText(QString::asprintf("处理完成 - 半径滤波"));

	temp.append(QString::asprintf("--- 半径滤波\n参数:"));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("查询点的半径邻域范围 = %.2f\n判断是否为离群点的阈值 = %d", F_RadiusSearch, F_MinNeighborsInRadius));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("处理前后参数对比: "));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("总点数: %ld -> %ld", cloud->height * cloud->width, cloud_filtered->height * cloud_filtered->width));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("剩余采样点占比 = %.3f%%", (float)((float)(cloud_filtered->width * cloud_filtered->height) / (float)(cloud->width * cloud->height)) * 100));
	PrintToLog(&temp);
	temp.clear();
	ui.TempOperationName->setText(QString::asprintf("半径滤波"));
	PointTempUIUpdater();
}

void QMainWindow1::on_butF_UniSampling_clicked()
{
	InitviaICP();
	ui.LogWindow->clear();
	label->setText(QString::asprintf("正在处理 - 均匀采样"));
	QString temp = "";

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	if (pcFileName.isEmpty())
	{
		temp.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		temp.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}

	pcl::UniformSampling<pcl::PointXYZ> US;
	US.setInputCloud(cloud);
	US.setRadiusSearch(F_UsRadius);// 设置滤波时创建球体的半径
	US.filter(*cloud_filtered);
	label->setText(QString::asprintf("处理完成 - 均匀采样"));

	temp.append(QString::asprintf("--- 均匀采样\n参数:"));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("滤波时采样球体的半径 = %.5f", F_UsRadius));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("处理前后参数对比: "));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("总点数: %ld -> %ld", cloud->height * cloud->width, cloud_filtered->height * cloud_filtered->width));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("剩余采样点占比 = %.3f%%", (float)((float)(cloud_filtered->width * cloud_filtered->height) / (float)(cloud->width * cloud->height)) * 100));
	PrintToLog(&temp);
	temp.clear();
	ui.TempOperationName->setText(QString::asprintf("半径滤波"));
	PointTempUIUpdater();
}
//ok

void QMainWindow1::on_butR_4PCS_clicked()
{
	QMessageBox::StandardButton yes1;
	yes1 = QMessageBox::warning(this, "警告", "该算法具有危险性，请仔细检查高级设置中 Delta 值是否配置恰当。\n请从小的 Delta 值开始处理，Delta 值过大会导致内存一直增加，可能会占满电脑所有内存然后程序崩掉。\n确定要继续执行吗？", QMessageBox::Yes | QMessageBox::No);
	if (yes1 != QMessageBox::Yes) return;
	cloud_filtered->clear();
	ui.LogWindow->clear();
	label->setText(QString::asprintf("正在处理 - 4PCS 配准"));
	QString temp = "";

	// Fill in the cloud data
	// Replace the path below with the path where you saved your file
	if (pcFileName.isEmpty())
	{
		temp.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		temp.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	if (pcFileNameS.isEmpty())
	{
		temp.append(QString::asprintf("没有打开辅助文件。\n需要辅助文件作为点云配准目标。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf2(pcFileNameS);
	QString suffix2 = inf2.suffix();
	if (suffix2 != "pcd" && suffix2 != "ply")
	{
		temp.append(QString::asprintf("当前打开的辅助文件不是 PCD 格式文件。\n请另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	
	fpcs.setInputSource(cloud);
	fpcs.setInputTarget(cloudS);
	fpcs.setApproxOverlap(R4_ApproxOverlap);//两点云重叠度
	fpcs.setDelta(R4_Delta);//Bunny
	fpcs.setMaxComputationTime(R4_MaxComputationTime);
	fpcs.setNumberOfSamples(R4_NumberOfSamples);
	Eigen::Matrix4f tras;

	for (int i = 0; i < 2; i++)
	{	
		cloud_temp->clear();
		clock_t start = clock();
		fpcs.align(*cloud_temp);
		clock_t end = clock();
		ui.LogWindow->clear();
		temp.append(QString::asprintf("--- 4PCS 配准\n实时数据:\n迭代次数 = %d", i+1));
		PrintToLog(&temp);
		temp.clear();
		temp.append(QString::asprintf("处理用时 = %.3f s", (double)(end - start) / (double)CLOCKS_PER_SEC));
		PrintToLog(&temp);
		temp.clear();
		temp.append(QString::asprintf("配准精度 = %.3f%%\n配准矩阵: [4x4]", 100.00 / (1.00 + (fpcs.getFitnessScore()))));
		PrintToLog(&temp);
		temp.clear();
		tras = fpcs.getFinalTransformation();
		for (int j = 0; j < 4; j++)
		{
			temp.append(QString::asprintf("%.3f %.3f %.3f %.3f", tras(j,0), tras(j, 1), tras(j, 2), tras(j, 3)));
			PrintToLog(&temp);
			temp.clear();
		}
	}
	
	tras = fpcs.getFinalTransformation();
	pcl::transformPointCloud(*cloud, *cloud_filtered, tras);

	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> points3 = vtkSmartPointer<vtkPoints>::New();
	int pointcount2 = 0;
	int pointcount3 = 0;
	for (const auto& point : *cloudS) //target cloud
	{

		pointcount2 = pointcount2 + 1;
		points2->InsertNextPoint(point.x, point.y, point.z);
	}
	for (const auto& point : *cloud_filtered)
	{

		pointcount3 = pointcount3 + 1;
		points3->InsertNextPoint(point.x, point.y, point.z);
	}
	//Visualization
	vtkSmartPointer<vtkPolyData> p1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> p2 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> p3 = vtkSmartPointer<vtkPolyData>::New();
	p1->SetPoints(points);
	p2->SetPoints(points2);
	p3->SetPoints(points3);
	vtkSmartPointer<vtkVertexGlyphFilter> g1 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g1->SetInputData(p1);
	vtkSmartPointer<vtkVertexGlyphFilter> g2 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g2->SetInputData(p2);
	vtkSmartPointer<vtkVertexGlyphFilter> g3 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g3->SetInputData(p3);
	g1->Update();
	g2->Update();
	g3->Update();
	vtkSmartPointer<vtkPolyDataMapper> sourceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sourceMapper->SetInputConnection(g1->GetOutputPort());

	vtkSmartPointer<vtkActor> sourceActor = vtkSmartPointer<vtkActor>::New();
	sourceActor->SetMapper(sourceMapper);
	sourceActor->GetProperty()->SetColor(0, 1, 0);
	sourceActor->GetProperty()->SetPointSize(regrp);
	int countG = sourceActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkPolyDataMapper> targetMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	targetMapper->SetInputConnection(g2->GetOutputPort());

	vtkSmartPointer<vtkActor> targetActor = vtkSmartPointer<vtkActor>::New();
	targetActor->SetMapper(targetMapper);
	targetActor->GetProperty()->SetColor(1, 0, 0);
	targetActor->GetProperty()->SetPointSize(regrp);
	int countR = targetActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkPolyDataMapper> solutionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	solutionMapper->SetInputConnection(g3->GetOutputPort());

	vtkSmartPointer<vtkActor> solutionActor = vtkSmartPointer<vtkActor>::New();
	solutionActor->SetMapper(solutionMapper);
	solutionActor->GetProperty()->SetColor(0, 0, 1);
	solutionActor->GetProperty()->SetPointSize(regrp);
	int countB = solutionActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->SetBackground(1.0, 1.0, 1.0);

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderer->AddActor(sourceActor);
	renderer->AddActor(targetActor);
	renderer->AddActor(solutionActor);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->SetSize(600, 600);
	renderWindow->Render();
	renderWindow->SetWindowName("4pcs Registration Preview");
	renderWindow->Render();
	renderWindow->Render();
	temp.append(QString::asprintf("--- 已打开展示页面"));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("配准源点集为绿色，配准目标为红色，配准结果为蓝色。"));
	PrintToLog(&temp);
	temp.clear();
	renderWindowInteractor->Start();
}

void QMainWindow1::on_butR_ICP_clicked()
{
	ui.LogWindow->clear();
	cloud_filtered->clear();
	label->setText(QString::asprintf("正在处理 - ICP 滤波"));
	QString temp = "";

	// Fill in the cloud data
	// Replace the path below with the path where you saved your file
	if (pcFileName.isEmpty())
	{
		temp.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf(pcFileName);
	QString suffix = inf.suffix();
	if (suffix != "pcd" && suffix != "ply")
	{
		temp.append(QString::asprintf("当前打开的文件不是 PCD 格式文件。\n请先另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	if (pcFileNameS.isEmpty())
	{
		temp.append(QString::asprintf("没有打开辅助文件。\n需要辅助文件作为点云配准目标。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	QFileInfo inf2(pcFileNameS);
	QString suffix2 = inf2.suffix();
	if (suffix2 != "pcd" && suffix2 != "ply")
	{
		temp.append(QString::asprintf("当前打开的辅助文件不是 PCD 格式文件。\n请另存为 PCD 文件后重试。"));
		PrintToLog(&temp);
		temp.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}

	fpcs1.setInputSource(cloud);
	fpcs1.setInputTarget(cloudS);
	if (isICPCustomizeEnable)
	{
		fpcs1.setMaxCorrespondenceDistance(RI_MaxCorrespondenceDistance);
		fpcs1.setTransformationEpsilon((float)(1.0 / (float)RI_TransformationEpsilon));
		fpcs1.setEuclideanFitnessEpsilon(RI_EuclideanFitnessEpsilon);
		fpcs1.setMaximumIterations(RI_MaximumIterations);
	}
	Eigen::Matrix4f tras;

	for (int i = 0; i < 9; i++)
	{
		cloud_temp->clear();
		clock_t start = clock();
		fpcs1.align(*cloud_temp);
		clock_t end = clock();
		ui.LogWindow->clear();
		temp.append(QString::asprintf("--- ICP 配准\n实时数据:\n迭代次数 = %d", i + 1));
		PrintToLog(&temp);
		temp.clear();
		temp.append(QString::asprintf("处理用时 = %.3f s", (double)(end - start) / (double)CLOCKS_PER_SEC));
		PrintToLog(&temp);
		temp.clear();
		if (fpcs1.hasConverged())
		{
			temp.append(QString::asprintf("配准有效。"));
			PrintToLog(&temp);
			temp.clear();
		}
		else
		{
			temp.append(QString::asprintf("配准无效。"));
			PrintToLog(&temp);
			temp.clear();
		}
		temp.append(QString::asprintf("配准精度 = %.3f%%\n配准矩阵: [4x4]", 100.00/(1.00+(fpcs1.getFitnessScore()))));
		PrintToLog(&temp);
		temp.clear();
		tras = fpcs1.getFinalTransformation();
		for (int j = 0; j < 4; j++)
		{
			temp.append(QString::asprintf("%.3f %.3f %.3f %.3f", tras(j, 0), tras(j, 1), tras(j, 2), tras(j, 3)));
			PrintToLog(&temp);
			temp.clear();
		}
	}
	tras = fpcs1.getFinalTransformation();
	pcl::transformPointCloud(*cloud, *cloud_filtered, tras);
	float Rmse = caculateRMSE(cloud, cloud_filtered);
	temp.append(QString::asprintf("配准误差为：%f", Rmse));
	PrintToLog(&temp);
	temp.clear();
	if (!isInitialized) isInitialized = 1;

	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> points3 = vtkSmartPointer<vtkPoints>::New();
	int pointcount2 = 0;
	int pointcount3 = 0;
	for (const auto& point : *cloudS) //target cloud
	{

		pointcount2 = pointcount2 + 1;
		points2->InsertNextPoint(point.x, point.y, point.z);
	}
	for (const auto& point : *cloud_filtered)
	{

		pointcount3 = pointcount3 + 1;
		points3->InsertNextPoint(point.x, point.y, point.z);
	}
	//Visualization
	vtkSmartPointer<vtkPolyData> p1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> p2 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> p3 = vtkSmartPointer<vtkPolyData>::New();
	p1->SetPoints(points);
	p2->SetPoints(points2);
	p3->SetPoints(points3);
	vtkSmartPointer<vtkVertexGlyphFilter> g1 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g1->SetInputData(p1);
	vtkSmartPointer<vtkVertexGlyphFilter> g2 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g2->SetInputData(p2);
	vtkSmartPointer<vtkVertexGlyphFilter> g3 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	g3->SetInputData(p3);
	g1->Update();
	g2->Update();
	g3->Update();
	vtkSmartPointer<vtkPolyDataMapper> sourceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sourceMapper->SetInputConnection(g1->GetOutputPort());

	vtkSmartPointer<vtkActor> sourceActor = vtkSmartPointer<vtkActor>::New();
	sourceActor->SetMapper(sourceMapper);
	sourceActor->GetProperty()->SetColor(0, 1, 0);
	sourceActor->GetProperty()->SetPointSize(regrp);
	int countG = sourceActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkPolyDataMapper> targetMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	targetMapper->SetInputConnection(g2->GetOutputPort());

	vtkSmartPointer<vtkActor> targetActor = vtkSmartPointer<vtkActor>::New();
	targetActor->SetMapper(targetMapper);
	targetActor->GetProperty()->SetColor(1, 0, 0);
	targetActor->GetProperty()->SetPointSize(regrp);
	int countR = targetActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkPolyDataMapper> solutionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	solutionMapper->SetInputConnection(g3->GetOutputPort());

	vtkSmartPointer<vtkActor> solutionActor = vtkSmartPointer<vtkActor>::New();
	solutionActor->SetMapper(solutionMapper);
	solutionActor->GetProperty()->SetColor(0, 0, 1);
	solutionActor->GetProperty()->SetPointSize(regrp);
	int countB = solutionActor->GetProperty()->GetReferenceCount();

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->SetBackground(1.0, 1.0, 1.0);

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderer->AddActor(sourceActor);
	renderer->AddActor(targetActor);
	renderer->AddActor(solutionActor);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->SetSize(600, 600);
	renderWindow->Render();
	renderWindow->SetWindowName("ICP Registration Preview");
	renderWindow->Render();
	renderWindow->Render();
	temp.append(QString::asprintf("--- 已打开展示页面"));
	PrintToLog(&temp);
	temp.clear();
	temp.append(QString::asprintf("配准源点集为绿色，配准目标为红色，配准结果为蓝色。"));
	PrintToLog(&temp);
	temp.clear();
	renderWindowInteractor->Start();
}

void QMainWindow1::on_butR_greedy_clicked()
{
	if (cloud->width == 0)
	{
		printer.append(QString::asprintf("当前点云为空或是非 PCD 文件。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	if (pcFileName.isEmpty())
	{
		printer.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&printer);
		printer.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	normals->clear();
	tree->setInputCloud(cloud);
	n11.setInputCloud(cloud);
	n11.setSearchMethod(tree);
	n11.setKSearch(RC_KS); //KSearch
	n11.compute(*normals);
	cloud_with_normals->clear();
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	tree2->setInputCloud(cloud_with_normals);
	gp3.setSearchRadius(RC_SearchRadius); //设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径【默认值 0】
	gp3.setMu(RC_Mu); //设置最近邻距离的乘子，以得到每个点的最终搜索半径【默认值 0】
	gp3.setMaximumNearestNeighbors(RC_MaximumNearestNeighbors); //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle((M_PI / 180) * RC_MaximumSurfaceAngle); // 90 degrees（pi）最大平面角
	gp3.setMinimumAngle((M_PI / 180) * RC_MinimumAngle); // 10 degrees 每个三角的最小角度
	gp3.setMaximumAngle((M_PI / 180) * RC_MaximumAngle); // 135 degrees 每个三角的最大角度
	gp3.setNormalConsistency(confident); //如果法向量一致，设置为true
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	ReconstPrev = "";
	ReconstTitle = "Greedy Triangular 3D Mesh Preview";
	ui.LogWindow->clear();
	printer.append(QString::asprintf("--- 贪婪投影三角化算法"));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("搜索最近邻点的最大数量 = %d", RC_MaximumNearestNeighbors));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("最大平面角 = %d°", RC_MaximumSurfaceAngle));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("三角形角度范围 = %d ~ %d", RC_MinimumAngle, RC_MaximumAngle));
	PrintToLog(&printer);
	printer.clear();
	gp3.reconstruct(triangles);
	gp3.reconstruct(PM);
	if (isViewPolyEnable1)
	{
		printer.append(QString::asprintf("目前显示的是保存为实际文件的展示效果。\n如有感到不适请关闭展示窗口，并调整高级设置。"));
		PrintToLog(&printer);
		printer.clear();
		ReconstPrev = "Actual File Vision";
		pcl::io::savePLYFile("temp.ply", triangles);
		VTKViewPly("temp.ply");
	}
	else
	{
		pcl::io::saveVTKFile("temp.vtk", triangles);
		VTKViewPly("temp.vtk");
	}
}

void QMainWindow1::on_butR_Cuvism_clicked()
{
	if (cloud->width == 0)
	{
		printer.append(QString::asprintf("当前点云为空或是非 PCD 文件。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	if (pcFileName.isEmpty())
	{
		printer.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&printer);
		printer.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	normals->clear();
	tree->setInputCloud(cloud);
	n11.setInputCloud(cloud);
	n11.setSearchMethod(tree);
	n11.setKSearch(RC_KS); //KSearch
	n11.compute(*normals);
	cloud_with_normals->clear();
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals
	tree2->setInputCloud(cloud_with_normals);
	mc.setIsoLevel(RC_IsoLevel);
	mc.setGridResolution(RC_GridResolution, RC_GridResolution, RC_GridResolution);
	mc.setPercentageExtendGrid(RC_PercentageExtendGrid);
	mc.setInputCloud(cloud_with_normals);
	mc.setSearchMethod(tree2);
	ReconstTitle = "Cubes Hoppe 3D Mesh Preview";
	ui.LogWindow->clear();
	printer.append(QString::asprintf("--- 三维立方体重建算法"));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("格栅分辨率 = %d", RC_GridResolution));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("Percentage = %.3f", RC_PercentageExtendGrid));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("ISO Level = %d", RC_IsoLevel));
	PrintToLog(&printer);
	printer.clear();
	mc.reconstruct(triangles);
	mc.reconstruct(PM);
	if (isViewPolyEnable2)
	{
		printer.append(QString::asprintf("目前显示的是保存为实际文件的展示效果。\n如有感到不适请关闭展示窗口，并调整高级设置。"));
		PrintToLog(&printer);
		printer.clear();
		ReconstPrev = "Actual File Vision";
		pcl::io::savePLYFile("temp.ply", triangles);
		VTKViewPly("temp.ply");
	}
	else
	{
		pcl::io::saveVTKFile("temp.vtk", triangles);
		VTKViewPly("temp.vtk");
	}
}

void QMainWindow1::on_butR_Poisson_clicked()
{
	if (cloud->width == 0)
	{
		printer.append(QString::asprintf("当前点云为空或是非 PCD 文件。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	if (pcFileName.isEmpty())
	{
		printer.append(QString::asprintf("没有打开文件。"));
		PrintToLog(&printer);
		printer.clear();
		label->setText(QString::asprintf("就绪"));
		return;
	}
	normals->clear();
	tree->setInputCloud(cloud);
	n11.setInputCloud(cloud);
	n11.setSearchMethod(tree);
	n11.setKSearch(RC_KS); //KSearch
	n11.compute(*normals);
	cloud_with_normals->clear();
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals
	tree2->setInputCloud(cloud_with_normals);
	pn.setConfidence(confident);
	pn.setIsoDivide(RC_IsoDivide);
	pn.setInputCloud(cloud_with_normals);
	pn.setSearchMethod(tree2);
	ReconstTitle = "Poisson Reconstruction 3D Mesh Preview";
	ui.LogWindow->clear();
	printer.append(QString::asprintf("--- 泊松重建算法"));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("ISO Division = %d", RC_IsoDivide));
	PrintToLog(&printer);
	printer.clear();
	pn.reconstruct(triangles);
	pn.reconstruct(PM);
	if (isViewPolyEnable3)
	{
		printer.append(QString::asprintf("目前显示的是保存为实际文件的展示效果。\n如有感到不适请关闭展示窗口，并调整高级设置。"));
		PrintToLog(&printer);
		printer.clear();
		ReconstPrev = "Actual File Vision";
		pcl::io::savePLYFile("temp.ply", triangles);
		VTKViewPly("temp.ply");
	}
	else
	{
		pcl::io::saveVTKFile("temp.vtk", triangles);
		VTKViewPly("temp.vtk");
	}
}

void QMainWindow1::on_butR_SaveFile_clicked()
{
	QString saveFile = QFileDialog::getSaveFileName(this, "另存为文件", "", "vtk 文件(*.vtk)");
	if (saveFile.isEmpty()) return;
	QByteArray ba1;
	ba1.append(saveFile);     //也可以 ba2 = s2.toLatin1();
	const char* c1 = ba1.data();
	pcl::io::saveVTKFile(c1, PM);
	PrintToLog(&saveFile);
	printer.append(QString::asprintf("文件保存成功。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_butR_SaveFile_2_clicked()
{
	QString saveFile = QFileDialog::getSaveFileName(this, "另存为文件", "", "ply 文件(*.ply)");
	if (saveFile.isEmpty()) return;
	QByteArray ba1;
	ba1.append(saveFile);     //也可以 ba2 = s2.toLatin1();
	const char* c1 = ba1.data();
	pcl::io::savePLYFile(c1, PM);
	PrintToLog(&saveFile);
	printer.append(QString::asprintf("文件保存成功。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_butR_SaveFile_3_clicked()
{
	QString saveFile = QFileDialog::getSaveFileName(this, "另存为文件", "", "obj 文件(*.obj)");
	if (saveFile.isEmpty()) return;
	QByteArray ba1;
	ba1.append(saveFile);     //也可以 ba2 = s2.toLatin1();
	const char* c1 = ba1.data();
	pcl::io::saveOBJFile(c1, PM);
	PrintToLog(&saveFile);
	printer.append(QString::asprintf("文件保存成功。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_butR_3DPreview_clicked()
{
	QString openFile = QFileDialog::getOpenFileName(this, "打开模型", "", "ply 文件(*.ply);;obj 文件(*.obj);;vtk 文件(*.vtk)");
	if (openFile.isEmpty()) return;
	ReconstTitle = "Previewing a local file...";
	ReconstPrev = "LOCAL FILE";
	PrintToLog(&openFile);
	printer.append(QString::asprintf("已打开模型预览窗口。"));
	PrintToLog(&printer);
	printer.clear();
	std::string ops = openFile.toStdString();
	VTKViewPly(ops);
}

void QMainWindow1::on_SaveFIlePCD_clicked()
{
	if (cloud->width == 0)
	{
		printer.append(QString::asprintf("当前点云为空或是未经处理。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	QString saveFile = QFileDialog::getSaveFileName(this, "另存为文件", "", "点云数据文件(*.pcd)");
	if (saveFile.isEmpty()) return;
	std::string ops = saveFile.toStdString();
	WritePcd(ops);
	PrintToLog(&saveFile);
	printer.append(QString::asprintf("文件保存成功。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_action_Save_triggered()
{
	if (cloud->width == 0)
	{
		printer.append(QString::asprintf("当前点云为空或是未经处理。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	on_SaveFIlePCD_clicked();
}

void QMainWindow1::on_HideAxis3_stateChanged(int arg1)
{
	Axis3 = ui.HideAxis3->isChecked();
}

void QMainWindow1::on_butC_Density_clicked()
{
	double Density = 0.00;
	if (cloud->width == 0)
	{
		printer.append(QString::asprintf("当前点云为空或是未经处理。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	if (!isCalAll)
	{
		Density = getMeanPointDensity(cloud, c_Maxdist, 1);
	}
	else
	{
		Density = compute_cloud_resolution(cloud);
	}
	printer.append(QString::asprintf("点云的平均距离密度为：%lf", Density));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("在单位距离构成的空间中，大概有 %.3lf 个点。", 1.00/Density));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_butR_mesh2pcd_clicked()
{
	QString tempFile = QFileDialog::getOpenFileName(this, "打开模型", "", "ply 文件(*.ply);;obj 文件(*.obj)");
	if (tempFile.isEmpty()) return;
	QString tempFile2 = QFileDialog::getSaveFileName(this, "保存文件的位置", "", "点云数据文件(*.pcd)");
	if (tempFile2.isEmpty()) return;
	QProcess p(0);
	QString y = "";
	y.clear();
	
	y.append(QString::asprintf("pcl_mesh2pcd -leaf_size %f ", m2p_leafsize));
	QByteArray ba1;
	ba1.append(tempFile);     //也可以 ba2 = s2.toLatin1();
	y.append(ba1);
	y.append(QString::asprintf(" "));
	QByteArray ba2;
	ba2.append(tempFile2);     //也可以 ba2 = s2.toLatin1();
	y.append(ba2);
	QByteArray ba3;
	ba3.append(y);     //也可以 ba2 = s2.toLatin1();
	const char* c1 = ba3.data();
	PrintToLog(&y);
	p.start("cmd", QStringList() << "/c" << c1);
	p.waitForStarted();
	p.waitForFinished();
	QString strTemp = QString::fromLocal8Bit(p.readAllStandardOutput());
	PrintToLog(&strTemp);

	printer.append(QString::asprintf("模型反转为 PCD 完成。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_butC_21_clicked()
{
	Eigen::Vector4f centroid;                    // 质心
	Eigen::Matrix3f covariance_matrix;           // 协方差矩阵
	pcl::computeMeanAndCovarianceMatrix(*cloud, covariance_matrix, centroid); //质心为齐次坐标（c0, c1, c2, 1）
	printer.append(QString::asprintf("点云的归一化3x3协方差矩阵为："));
	PrintToLog(&printer);
	printer.clear();
	for (int i = 0; i < 3; i++)
	{
		printer.append(QString::asprintf("%f %f %f", covariance_matrix(i,0), covariance_matrix(i,1), covariance_matrix(i,2)));
		PrintToLog(&printer);
		printer.clear();
	}
	printer.append(QString::asprintf("点云质心为："));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("%f %f %f %f", centroid(0), centroid(1), centroid(2), centroid(3)));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_SetbgColor3_clicked()
{
	QColor color(rb3, gb3, bb3);
	QColor color2 = QColorDialog::getColor(color, this, "选择三维窗口背景色");
	if (!color2.isValid()) return;
	color = color2;
	color.getRgb(&rb3, &gb3, &bb3);
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, color);
	ui.bgc01->setPalette(temppal);
}

void QMainWindow1::on_SetMeshColor3_clicked()
{
	QColor color(rb32, gb32, bb32);
	QColor color2 = QColorDialog::getColor(color, this, "选择三维窗口模型颜色");
	if (!color2.isValid()) return;
	color = color2;
	color.getRgb(&rb32, &gb32, &bb32);
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, color);
	ui.bgc02->setPalette(temppal);
}

void QMainWindow1::on_HideAxisW_stateChanged(int arg1)
{
	AxisW = ui.HideAxisW->isChecked();
}

void QMainWindow1::on_HideAxisV_stateChanged(int arg1)
{
	AxisV = ui.HideAxisV->isChecked();
}

void QMainWindow1::on_SetbgColorV_clicked()
{
	QColor color(vtkbr, vtkbg, vtkbb);
	QColor color2 = QColorDialog::getColor(color, this, "选择内部预览窗口颜色");
	if (!color2.isValid()) return;
	color = color2;
	color.getRgb(&vtkbr, &vtkbg, &vtkbb);
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, color);
	ui.VTKBColor->setPalette(temppal);
}

void QMainWindow1::on_SetPointColorV_clicked()
{
	QColor color(vtkdr, vtkdg, vtkdb);
	QColor color2 = QColorDialog::getColor(color, this, "选择内部预览点云颜色");
	if (!color2.isValid()) return;
	color = color2;
	color.getRgb(&vtkdr, &vtkdg, &vtkdb);
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, color);
	ui.VTKDColor->setPalette(temppal);
}

void QMainWindow1::on_SetbgColorW_clicked()
{
	QColor color(acbr, acbg, acbb);
	QColor color2 = QColorDialog::getColor(color, this, "选择外部预览窗口颜色");
	if (!color2.isValid()) return;
	color = color2;
	color.getRgb(&acbr, &acbg, &acbb);
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, color);
	ui.bgc03->setPalette(temppal);
}

void QMainWindow1::on_SetPointColorW1_clicked()
{
	QColor color(acdr1, acdg1, acdb1);
	QColor color2 = QColorDialog::getColor(color, this, "选择外部预览点云 1 颜色");
	if (!color2.isValid()) return;
	color = color2;
	color.getRgb(&acdr1, &acdg1, &acdb1);
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, color);
	ui.bgc04->setPalette(temppal);
}

void QMainWindow1::on_SetPointColorW2_clicked()
{
	QColor color(acdr2, acdg2, acdb2);
	QColor color2 = QColorDialog::getColor(color, this, "选择外部预览点云 2 颜色");
	if (!color2.isValid()) return;
	color = color2;
	color.getRgb(&acdr2, &acdg2, &acdb2);
	QPalette temppal;
	temppal.setColor(QPalette::WindowText, color);
	ui.bgc05->setPalette(temppal);
}

void QMainWindow1::on_SetApplyPointSizeW_clicked()
{
	int ds1 = ui.pointsz1->text().toInt();
	ps1 = ds1;
	int ds2 = ui.pointsz2->text().toInt();
	ps2 = ds2;
	printer.append(QString::asprintf("已应用外部窗口点云数据大小。"));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("点云 1 数据大小 = %d\n点云 2 数据大小 = %d", ps1, ps2));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_VTKWApply_clicked()
{
	int ds1 = ui.pointszV->text().toInt();
	psv = ds1;
	int ds2 = ui.pointszVR->text().toInt();
	psvr = ds2;
	printer.append(QString::asprintf("已应用内部窗口点云数据大小。"));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("点云数据大小 = %d\n三维重建示例点云数据大小 = %d", psv, psvr));
	PrintToLog(&printer);
	printer.clear();
	if (points == 0)
	{
		printer.append(QString::asprintf("没有原始数据，窗口重载失败。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	on_DataPreview_clicked();
	printer.append(QString::asprintf("窗口重载成功。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_butF_OverwriteUI_R_clicked()
{
	if (cloud_filtered->width == 0 || cloud_filtered->height == 0)
	{
		printer.append(QString::asprintf("没有可供保存的处理结果。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	cloud->clear();
	*cloud = *cloud_filtered;
	cloud_filtered->clear();
	UpdateCloud();
	printer.append(QString::asprintf("数据覆盖成功。"));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_SetPointSizerp_clicked()
{
	int ds1 = ui.pointsrp->text().toInt();
	regrp = ds1;
	printer.append(QString::asprintf("已应用配准窗口点云数据大小。"));
	PrintToLog(&printer);
	printer.clear();
	printer.append(QString::asprintf("点云数据大小 = %d", regrp));
	PrintToLog(&printer);
	printer.clear();
}

void QMainWindow1::on_CloseFile_clicked()
{
	pcFileName.clear();
	ui.FileDir->setText(QString::asprintf("File Name"));
	ui.TotalDots->setText(QString::asprintf("0"));
	ui.FileType->setText(QString::asprintf("null"));
	printer.append(QString::asprintf("文件已关闭。"));
	PrintToLog(&printer);
	printer.clear();
	ui.CloseFile->setEnabled(0);
}

void QMainWindow1::on_CloseFile_Sub_clicked()
{
	pcFileNameS.clear();
	ui.FileDir_Sub->setText(QString::asprintf("Sub File Name"));
	ui.TotalDots_2->setText(QString::asprintf("0"));
	ui.FileType_2->setText(QString::asprintf("null"));
	printer.append(QString::asprintf("文件已关闭。"));
	PrintToLog(&printer);
	printer.clear();
	ui.CloseFile_Sub->setEnabled(0);
}

void QMainWindow1::on_CloseFile_All_clicked()
{
	on_CloseFile_clicked();
	on_CloseFile_Sub_clicked();
	ui.CloseFile_All->setEnabled(0);
}

void QMainWindow1::on_action_Save_2_triggered()
{
	if (cloud->width == 0)
	{
		printer.append(QString::asprintf("当前点云为空或是未经处理。"));
		PrintToLog(&printer);
		printer.clear();
		return;
	}
	QString saveFile = QFileDialog::getSaveFileName(this, "另存为文件", "", "点云数据文件(*.ply)");
	if (saveFile.isEmpty()) return;
	std::string ops = saveFile.toStdString();
	WritePly(ops);
	PrintToLog(&saveFile);
	printer.append(QString::asprintf("文件保存成功。"));
	PrintToLog(&printer);
	printer.clear();
}