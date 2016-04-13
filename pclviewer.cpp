#include <pcl/visualization/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include "pclviewer.h"
#include "ui_pclviewer.h" //Will be generated when building...
#include "cloudemitterreceiver.h"
#include <QFileDialog>
#include "algos/downsampler.h"
#include "algos/planeextractor.h"


PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->setCameraPosition(-0.65, -0.08, -1.7, 0, -1, 0);
    viewer->setCameraFieldOfView(3.1415/6);
    viewer->setCameraClipDistances(1.32, 4.8);
    ui->qvtkWidget->update ();

    // Set the number of columns in the tree
    ui->treeWidget->hide();
    ui->treeWidget->setColumnCount(2);
    ui->treeWidget->header()->hideSection(1); // Contains IDs
    ui->treeWidget->header()->hide();
    ui->treeWidget->setUniformRowHeights(true); //Optimizations

    captureDevice = NULL;
}

void PCLViewer::connectToCloud(CloudEmitter *input) {
    connect(input, SIGNAL(newCloudMessage(const CloudMessage::ConstPtr)),
            this, SLOT(receiveCloudMessage(const CloudMessage::ConstPtr)));
}

void PCLViewer::setCaptureDevice(Capture* device) {
    captureDevice = device;
}

void PCLViewer::receiveCloudMessage(const CloudMessage::ConstPtr &message) {
    treeShowMessageSource(message);
    displayClouds(message);
    previousMessage = message;
}

void PCLViewer::displayClouds(const CloudMessage::ConstPtr &message) {
    for(int i=0; i<message->clouds.size(); i++) {
        Cloud::Ptr cloud = message->clouds[i];

        // Unique name for cloud
        const std::string cloudName = QString::number(message->emitterID).toStdString() +
                QString::number(i).toStdString() +
                cloud->cloudName.toStdString();

        // Dirty but simplifies choosing colors for clouds in the processing part of this tool...
        cloud->cloudColor.setInputCloud(cloud->pointCloud);

        // Add or update point cloud in PCL viewer
        if (viewer->updatePointCloud(cloud->pointCloud, cloud->cloudColor, cloudName) == false)
            viewer->addPointCloud(cloud->pointCloud, cloud->cloudColor, cloudName);

        // Show cloud in tree widget (as a child of the emitter)
        treeShowCloud(cloud, message->emitterID);

        // Remove old object clouds (might be unnecessary, but we have no way of knowing currently...
        for(int j=0; j<100; j++) {
            std::string objectName = cloudName + QString::number(j).toStdString();
            viewer->removePointCloud(objectName);
        }

        // Add or update objects contained in pointcloud (if any)
        if(!cloud->objects.empty()) {

            PointCloudT::Ptr objectCloud(new PointCloudT);
            pcl::ExtractIndices<PointT> extractor(true);
            extractor.setInputCloud(cloud->pointCloud);

            for (int j=0; j<cloud->objects.size(); j++) {
                extractor.setIndices(cloud->objects[j]->indices);
                extractor.filter(*objectCloud);

                std::string objectName = cloudName + QString::number(j).toStdString();

                cloud->objects[j]->objectColor.setInputCloud(objectCloud);

                // Add or update object in PCL viewer
                if (viewer->updatePointCloud(objectCloud, cloud->objects[j]->objectColor, objectName) == false)
                    viewer->addPointCloud(objectCloud, cloud->objects[j]->objectColor, objectName);

                // Show object in tree widget (as a child of the cloud)
                treeShowCloud(cloud, message->emitterID);
            }
        }
    }

    // Update PCL viewer and qvtkWidget
    ui->qvtkWidget->update ();
}

void PCLViewer::treeShowMessageSource(const CloudMessage::ConstPtr &message) {
    // Try to find emitter to update it
    QTreeWidgetItem* item = treeFindItem(ui->treeWidget, message->emitterID);

    // Otherwise create a new item
    if (item==NULL) {
        item = new QTreeWidgetItem(ui->treeWidget);
        item->setText(1,QString::number(message->emitterID));
        item->setText(0,message->emitterName);
        item->addChild(new QTreeWidgetItem()); // Info
        item->addChild(new QTreeWidgetItem()); // Clouds
        item->child(0)->setText(0, "Information");
        item->child(1)->setText(0, "Clouds");
    }

    // Get emitter information node
    QTreeWidgetItem* infoItem = item->child(0);

    // Update emitter information items
    treeUpdateInfo(infoItem, message->emitterInfo);
}

void PCLViewer::treeShowCloud(const Cloud::ConstPtr &cloud, const int emitterID) {
    // Find "Clouds" root item in emitter item
    QTreeWidgetItem *cloudsRootItem = treeFindChildItem(ui->treeWidget,
                                                        treeFindItem(ui->treeWidget, emitterID),
                                                        "Clouds");

    // Find specific cloud item in "Clouds" root item
    QTreeWidgetItem *cloudItem = treeFindChildItem(ui->treeWidget,
                                                   cloudsRootItem,
                                                   cloud->cloudName);

    // Should always hold...
    if (cloudsRootItem!=NULL) {
        // Add this cloud to the Clouds root item
        if(cloudItem==NULL) {
            cloudItem = new QTreeWidgetItem(cloudsRootItem);
            cloudItem->setText(0, cloud->cloudName);
            cloudItem->setBackgroundColor(0, QColor(cloud->cloudColor.r__, cloud->cloudColor.g__, cloud->cloudColor.b__));
            cloudItem->addChild(new QTreeWidgetItem()); // Info
            cloudItem->addChild(new QTreeWidgetItem()); // Objects
            cloudItem->child(0)->setText(0, "Information");
            cloudItem->child(1)->setText(0, "Objects");
        }

        // Get cloud information node
        QTreeWidgetItem* infoItem = cloudItem->child(0);
        // Update cloud information items
        treeUpdateInfo(infoItem, cloud->cloudInfo);

        // Get cloud information node
        QTreeWidgetItem* objectsItem = cloudItem->child(1);
        // Update cloud object items
        treeUpdateObjects(objectsItem, cloud->objects);
    }
}

QTreeWidgetItem* PCLViewer::treeFindItem(const QTreeWidget* tree, const int ID) {
    QList<QTreeWidgetItem*> list = tree->findItems(QString::number(ID), Qt::MatchExactly, 1);
    if(list.size()==1)
        return list.at(0);
    else if(list.size()==0)
        return NULL;
    else {
        std::cerr << "[pclviewer.cpp] Error: treeItem with ID \""
                     + QString::number(ID).toStdString() + "\" is not unique!!." << std::endl;
        std::cerr << "[pclviewer.cpp] Returning first item found." << std::endl;

        return list.at(0);
    }
}

QList<QTreeWidgetItem*> PCLViewer::treeFindItem(const QTreeWidget* tree, QString name) {
    return tree->findItems(name, Qt::MatchExactly|Qt::MatchRecursive, 0);
}

QTreeWidgetItem* PCLViewer::treeFindChildItem(const QTreeWidget* tree, const QTreeWidgetItem* parent, QString childName) {
    if(parent!=NULL) {
        QList<QTreeWidgetItem*> list = treeFindItem(tree, childName);
        for(int i=0; i<list.size(); i++)
        {
            QTreeWidgetItem* child = list.at(i);
            if(child->parent()== parent)
                return child;
        }
    }
    return NULL;
}

void PCLViewer::treeUpdateInfo(QTreeWidgetItem* infoItem, const QVector<QString> &infos) {
    // Add or remove childs if necessary
    while(infoItem->childCount() != infos.size()) {
        if (infoItem->childCount() < infos.size())
            infoItem->addChild(new QTreeWidgetItem());
        else
            infoItem->removeChild(infoItem->child(0));
    }

    // Update cloud information items
    int i = 0;
    for(int j=0; j<infos.size(); j++) {
        QString text = infos.at(j);
        QTreeWidgetItem *subItem = infoItem->child(i);
        subItem->setText(0,text);
        i++;
    }
}

void PCLViewer::treeUpdateObjects(QTreeWidgetItem* objectsItem, const QVector<Object::Ptr> &objects) {
    // Add or remove childs if necessary
    while(objectsItem->childCount() != objects.size()) {
        if (objectsItem->childCount() < objects.size())
            objectsItem->addChild(new QTreeWidgetItem());
        else
            objectsItem->removeChild(objectsItem->child(0));
    }

    // Update cloud information items
    int i = 0;
    for(int j=0; j<objects.size(); j++) {
        QString text = objects[j]->objectName;
        QTreeWidgetItem *subItem = objectsItem->child(i);
        subItem->setText(0,text);
        subItem->setBackgroundColor(0, QColor(objects[j]->objectColor.r__, objects[j]->objectColor.g__, objects[j]->objectColor.b__));
        i++;
    }
}

PCLViewer::~PCLViewer ()
{
    delete ui;
}

void PCLViewer::on_startPushbutton_released()
{
    if(captureDevice == NULL) {
        if(!ui->useFile->isChecked()) {
            // Create a new grabber for OpenNI devices
            grabber = new pcl::OpenNIGrabber();
            // Create a Capture object running the grabber in a seperate thread
            captureDevice = new Capture(grabber);
        }
        else {
            // Create a Capture object simulating a camera using the file
            captureDevice = new Capture(fileName);
        }

        /*--------------------------------*/
        /* Setup and link algorithms here */
        /*--------------------------------*/

        // Downsample cloud
        //Downsampler* downsampler = new Downsampler();
        //downsampler->connectToCloud(camera);

        // Extract plane
        PlaneExtractor* planeExtractor = new PlaneExtractor();
        planeExtractor->connectToCloud(captureDevice);

        /*------------------------------------------------------*/
        /* Link the output of the algorithms to the viewer here */
        /*------------------------------------------------------*/

        //w.connectToCloud(camera); // View source point cloud
        //w.connectToCloud(downsampler); // View downsampled source cloud
        connectToCloud(planeExtractor); // View extracted plane
    }

    if(captureDevice!=NULL) {
        if(!captureDevice->isRunning()) {
            captureDevice->run();
            ui->startPushbutton->setText("Stop capture");
        }
        else {
            captureDevice->stop();
            ui->startPushbutton->setText("Start capture");
        }
    }
}

void PCLViewer::on_exportButton_released()
{
    // Very crappy. Need to think about a better way to get clouds in the viewer --> through the TreeView, once it's usable.
    if(previousMessage!=NULL) {
        for(int i=0; i<previousMessage->clouds.size(); i++) {
            for(int j=0; j<previousMessage->clouds[i]->objects.size(); j++) {
                if(previousMessage->clouds[i]->objects[j]->objectName == "Objects on table") {
                    pcl::ExtractIndices<PointT> extractor(true);
                    extractor.setInputCloud(previousMessage->clouds[i]->pointCloud);
                    extractor.setIndices(previousMessage->clouds[i]->objects[j]->indices);
                    PointCloudT::Ptr objectCloud(new PointCloudT);
                    extractor.filter(*objectCloud);
                    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("PCD file (*.pcd)"));
                    pcl::PCDWriter w;
                    w.writeBinaryCompressed(fileName.toStdString(), *objectCloud);
                }
            }
        }
    }
}

void PCLViewer::on_useFile_toggled(bool checked)
{
    if(checked) {
        fileName = QFileDialog::getOpenFileName(this, tr("Open Pointcloud"), "", tr("PCD files (*.pcd)"));
        ui->useFile->setDisabled(true);
    }
}
