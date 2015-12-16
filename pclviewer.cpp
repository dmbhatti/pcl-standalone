#include <pcl/visualization/common/common.h>
#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include "cloudemitterreceiver.h"

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    // Set the number of columns in the tree
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
}

void PCLViewer::displayClouds(const CloudMessage::ConstPtr &message) {

    for(int i=0; i<message->clouds.size(); i++) {

        Cloud::Ptr cloud = message->clouds.at(i);

        // Unique name for cloud
        const std::string cloudName = QString::number(message->emitterID).toStdString() +
                cloud->cloudName.toStdString();

        // Dirty but simplifies choosing colors for clouds in the processing part of this tool...
        cloud->cloudColor.setInputCloud(cloud->pointCloud);

        // Add or update point cloud in PCL viewer
        if (viewer->updatePointCloud(cloud->pointCloud, cloud->cloudColor, cloudName) == false)
            viewer->addPointCloud(cloud->pointCloud, cloud->cloudColor, cloudName);

        // Show cloud in tree widget (as a child of the emitter)
        treeShowCloud(cloud, message->emitterID);
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
        }

        // Update cloud information items
        treeUpdateInfo(cloudItem, cloud->cloudInfo);
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

PCLViewer::~PCLViewer ()
{
    delete ui;
}

void PCLViewer::on_startPushbutton_released()
{
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


