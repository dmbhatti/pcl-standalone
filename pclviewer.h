#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QObject>
#include <QMainWindow>

// Point Cloud Library
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QTreeWidget>


#include "globals.h"
#include "cloudemitterreceiver.h"
#include "capture.h"

namespace Ui
{
class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer (QWidget *parent = 0);
    ~PCLViewer ();

    void connectToCloud(CloudEmitter *input);

    void setCaptureDevice(Capture* device);

public slots:
    /** Receives cloud messages. */
    void receiveCloudMessage(const CloudMessage::ConstPtr &message);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private slots:
    void on_startPushbutton_released();

    void on_exportButton_released();

    void on_useFile_toggled(bool checked);

private:
    void displayClouds(const CloudMessage::ConstPtr &message);
    void treeShowMessageSource(const CloudMessage::ConstPtr &message);
    void treeShowCloud(const Cloud::ConstPtr &cloud, const int emitterID);

    QTreeWidgetItem* treeFindItem(const QTreeWidget* tree, const int ID);
    QList<QTreeWidgetItem*> treeFindItem(const QTreeWidget* tree, QString name);
    QTreeWidgetItem* treeFindChildItem(const QTreeWidget* tree, const QTreeWidgetItem* parent, QString childName);

    void treeUpdateInfo(QTreeWidgetItem* infoItem, const QVector<QString> &infos);
    void treeUpdateObjects(QTreeWidgetItem* objectsItem, const QVector<Object::Ptr> &objects);

    Ui::PCLViewer *ui;

    pcl::Grabber* grabber;
    Capture* captureDevice;
    QString fileName;

    CloudMessage::ConstPtr previousMessage;
};

#endif // PCLVIEWER_H
