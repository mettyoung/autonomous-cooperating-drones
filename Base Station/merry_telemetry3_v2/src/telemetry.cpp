#include "telemetry.h"
#include "ui_telemetry.h"

Telemetry::Telemetry()
{
    ui = new Ui::Telemetry();
    ui->setupUi(this);
    this->zero_reference = ros::Time::now().toSec();

    timer = new QTimer();

    init_subscribers();
    init_publishers();

    init_plot_graphs();
    set_axis_labels();
    enable_graph_legends(true);
    set_interactions_settings(true, true);

    // Set y-axis range
    ui->plot_height1->yAxis->setRange(0, 4);
    ui->plot_height2->yAxis->setRange(0, 4);
    ui->plot_yaw1->yAxis->setRange(-180, 180);
    ui->plot_yaw2->yAxis->setRange(-180, 180);

    timer->start(100);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(run()));
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(actualCoordinates()));
}

Telemetry::~Telemetry()
{
    delete ui;
}

void Telemetry::run()
{
    ros::spinOnce();
}

void Telemetry::init_subscribers()
{
    // Subscribers for the desired sensor readings QUAD1
    sub_thro1 = n_gui.subscribe("quad1/goal/height", 1, &Telemetry::updateThrottleTableDesired1, this);
    sub_yaw1 = n_gui.subscribe("quad1/goal/yaw", 1, &Telemetry::updateYawTableDesired1, this);

    // Subscribers for the desired sensor readings QUAD2
    sub_thro2 = n_gui.subscribe("quad2/goal/height", 1, &Telemetry::updateThrottleTableDesired2, this);
    sub_yaw2 = n_gui.subscribe("quad2/goal/yaw", 1, &Telemetry::updateYawTableDesired2, this);

    // Subscribers for the actual sensor readings QUAD1
    sub_thro_tar1 = n_gui.subscribe("quad1/actual/height", 1, &Telemetry::updateThrottleTableActual1, this);
    sub_yaw_tar1 = n_gui.subscribe("quad1/actual/yaw", 1, &Telemetry::updateYawTableActual1, this);

    // Subscribers for the actual sensor readings QUAD2
    sub_thro_tar2 = n_gui.subscribe("quad2/actual/height", 1, &Telemetry::updateThrottleTableActual2, this);
    sub_yaw_tar2 = n_gui.subscribe("quad2/actual/yaw", 1, &Telemetry::updateYawTableActual2, this);

    // Subsribers for the battery level readings
    sub_batt1 = n_gui.subscribe("quad1/batt", 1, &Telemetry::adjustBatteryLevel1, this);
    sub_batt2 = n_gui.subscribe("quad2/batt", 1, &Telemetry::adjustBatteryLevel2, this);

//    sub_coordinates_tar1 = n_gui.subscribe("quad1/goal/waypoint", 1, &Telemetry::updateCoordinatesTableDesired1, this);
//    sub_coordinates_tar2 = n_gui.subscribe("quad2/goal/waypoint", 1, &Telemetry::updateCoordinatesTableDesired2, this);

    PIDO[0][0][0] = n_gui.subscribe("quad1/pitch/pterm", 1, &Telemetry::updatePTermPitch0, this);
    PIDO[0][0][1] = n_gui.subscribe("quad1/pitch/iterm", 1, &Telemetry::updateITermPitch0, this);
    PIDO[0][0][2] = n_gui.subscribe("quad1/pitch/dterm", 1, &Telemetry::updateDTermPitch0, this);
    PIDO[0][0][3] = n_gui.subscribe("quad1/pitch/output", 1, &Telemetry::updateOutputPitch0, this);

    PIDO[0][1][0] = n_gui.subscribe("quad1/roll/pterm", 1, &Telemetry::updatePTermRoll0, this);
    PIDO[0][1][1] = n_gui.subscribe("quad1/roll/iterm", 1, &Telemetry::updateITermRoll0, this);
    PIDO[0][1][2] = n_gui.subscribe("quad1/roll/dterm", 1, &Telemetry::updateDTermRoll0, this);
    PIDO[0][1][3] = n_gui.subscribe("quad1/roll/output", 1, &Telemetry::updateOutputRoll0, this);

    PIDO[1][0][0] = n_gui.subscribe("quad2/pitch/pterm", 1, &Telemetry::updatePTermPitch1, this);
    PIDO[1][0][1] = n_gui.subscribe("quad2/pitch/iterm", 1, &Telemetry::updateITermPitch1, this);
    PIDO[1][0][2] = n_gui.subscribe("quad2/pitch/dterm", 1, &Telemetry::updateDTermPitch1, this);
    PIDO[1][0][3] = n_gui.subscribe("quad2/pitch/output", 1, &Telemetry::updateOutputPitch1, this);

    PIDO[1][1][0] = n_gui.subscribe("quad2/roll/pterm", 1, &Telemetry::updatePTermRoll1, this);
    PIDO[1][1][1] = n_gui.subscribe("quad2/roll/iterm", 1, &Telemetry::updateITermRoll1, this);
    PIDO[1][1][2] = n_gui.subscribe("quad2/roll/dterm", 1, &Telemetry::updateDTermRoll1, this);
    PIDO[1][1][3] = n_gui.subscribe("quad2/roll/output", 1, &Telemetry::updateOutputRoll1, this);
}

void Telemetry::init_publishers()
{    // Publishers for testing control
    pub_height = n_gui.advertise<merryMsg::Msg>("quad1/goal/height", 1000);
    pub_yaw = n_gui.advertise<merryMsg::Msg>("quad1/goal/yaw", 1000);
    pub_start = n_gui.advertise<std_msgs::Bool>("quad1/start", 1);
    pub_arm = n_gui.advertise<std_msgs::Bool>("quad1/arm", 1);

    pub_height2 = n_gui.advertise<merryMsg::Msg>("quad2/goal/height", 1000);
    pub_yaw2 = n_gui.advertise<merryMsg::Msg>("quad2/goal/yaw", 1000);
    pub_start2 = n_gui.advertise<std_msgs::Bool>("quad2/start", 1);
    pub_arm2 = n_gui.advertise<std_msgs::Bool>("quad2/arm", 1);

    pub_land1 = n_gui.advertise<std_msgs::Bool>("quad1/land", 1);
    pub_land2 = n_gui.advertise<std_msgs::Bool>("quad2/land", 1);
}

void Telemetry::init_plot_graphs()
{
    ui->txt_height->setText("0");
    ui->txt_yaw->setText("0");

    // QUAD1
    // Graph of desired height values.
    ui->plot_height1->addGraph();
    ui->plot_height1->graph(0)->setPen(QPen(Qt::green));
    ui->plot_height1->graph(0)->setName("Goal height");
    ui->plot_height1->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle));
    // Graph of desired  yaw values.
    ui->plot_yaw1->addGraph();
    ui->plot_yaw1->graph(0)->setPen(QPen(Qt::green));
    ui->plot_yaw1->graph(0)->setName("Goal yaw");
    // Graph of X goals.
    ui->plot_x1->addGraph();
    ui->plot_x1->graph(0)->setPen(QPen(Qt::green));
    ui->plot_x1->graph(0)->setName("P Term");
    // Graph of Y goals.
    ui->plot_y1->addGraph();
    ui->plot_y1->graph(0)->setPen(QPen(Qt::green));
    ui->plot_y1->graph(0)->setName("P Term");

    // QUAD1
    // Graph of actual height values.
    ui->plot_height1->addGraph();
    ui->plot_height1->graph(1)->setPen(QPen(Qt::red));
    ui->plot_height1->graph(1)->setName("Actual height");
    // Graph of actual yaw values.
    ui->plot_yaw1->addGraph();
    ui->plot_yaw1->graph(1)->setPen(QPen(Qt::red));
    ui->plot_yaw1->graph(1)->setName("Actual yaw");
    // Graph of X actuals.
    ui->plot_x1->addGraph();
    ui->plot_x1->graph(1)->setPen(QPen(Qt::red));
    ui->plot_x1->graph(1)->setName("I Term");
    // Graph of Y actuals.
    ui->plot_y1->addGraph();
    ui->plot_y1->graph(1)->setPen(QPen(Qt::red));
    ui->plot_y1->graph(1)->setName("I Term");

    // QUAD1
    // Difference of desired - actual.
    ui->plot_height1->addGraph();
    ui->plot_height1->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_height1->graph(2)->setName("Difference");
    ui->plot_yaw1->addGraph();
    ui->plot_yaw1->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_yaw1->graph(2)->setName("Difference");

    // QUAD2
    // Graph of desired height values.
    ui->plot_height2->addGraph();
    ui->plot_height2->graph(0)->setPen(QPen(Qt::green));
    ui->plot_height2->graph(0)->setName("Goal height");
    ui->plot_height2->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle));
    // Graph of desired yaw values.
    ui->plot_yaw2->addGraph();
    ui->plot_yaw2->graph(0)->setPen(QPen(Qt::green));
    ui->plot_yaw2->graph(0)->setName("Goal yaw");
    // Graph of X goals.
    ui->plot_x2->addGraph();
    ui->plot_x2->graph(0)->setPen(QPen(Qt::green));
    ui->plot_x2->graph(0)->setName("P Term");
    // Graph of Y goals.
    ui->plot_y2->addGraph();
    ui->plot_y2->graph(0)->setPen(QPen(Qt::green));
    ui->plot_y2->graph(0)->setName("P Term");

    // QUAD2
    // Graph of actual height values.
    ui->plot_height2->addGraph();
    ui->plot_height2->graph(1)->setPen(QPen(Qt::red));
    ui->plot_height2->graph(1)->setName("Actual height");
    // Graph of actual yaw values.
    ui->plot_yaw2->addGraph();
    ui->plot_yaw2->graph(1)->setPen(QPen(Qt::red));
    ui->plot_yaw2->graph(1)->setName("Actual yaw");
    // Graph of X actuals.
    ui->plot_x2->addGraph();
    ui->plot_x2->graph(1)->setPen(QPen(Qt::red));
    ui->plot_x2->graph(1)->setName("I Term");
    // Graph of Y actuals.
    ui->plot_y2->addGraph();
    ui->plot_y2->graph(1)->setPen(QPen(Qt::red));
    ui->plot_y2->graph(1)->setName("I Term");

    // QUAD2
    // Difference of desired - actual.
    ui->plot_height2->addGraph();
    ui->plot_height2->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_height2->graph(2)->setName("Difference");
    ui->plot_yaw2->addGraph();
    ui->plot_yaw2->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_yaw2->graph(2)->setName("Difference");

    // P Error for X and Y position
    ui->plot_x1->addGraph();
    ui->plot_x1->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_x1->graph(2)->setName("D Term");
    ui->plot_y1->addGraph();
    ui->plot_y1->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_y1->graph(2)->setName("D Term");

    ui->plot_x2->addGraph();
    ui->plot_x2->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_x2->graph(2)->setName("D Term");
    ui->plot_y2->addGraph();
    ui->plot_y2->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_y2->graph(2)->setName("D Term");
    // D Error for X and Y position
    ui->plot_x1->addGraph();
    ui->plot_x1->graph(3)->setPen(QPen(Qt::yellow));
    ui->plot_x1->graph(3)->setName("Output");
    ui->plot_y1->addGraph();
    ui->plot_y1->graph(3)->setPen(QPen(Qt::yellow));
    ui->plot_y1->graph(3)->setName("Output");

    ui->plot_x2->addGraph();
    ui->plot_x2->graph(3)->setPen(QPen(Qt::yellow));
    ui->plot_x2->graph(3)->setName("Output");
    ui->plot_y2->addGraph();
    ui->plot_y2->graph(3)->setPen(QPen(Qt::yellow));
    ui->plot_y2->graph(3)->setName("Output");
}

void Telemetry::set_axis_labels()
{
    // Set axis labels.
    ui->plot_height1->xAxis->setLabel("Time (s)");
    ui->plot_height2->xAxis->setLabel("Time (s)");
    ui->plot_height1->yAxis->setLabel("Height (m)");
    ui->plot_height2->yAxis->setLabel("Height (m)");
    ui->plot_yaw1->xAxis->setLabel("Time (s)");
    ui->plot_yaw2->xAxis->setLabel("Time (s)");
    ui->plot_yaw1->yAxis->setLabel("Yaw (deg)");
    ui->plot_yaw2->yAxis->setLabel("Yaw (deg)");
    ui->plot_x1->xAxis->setLabel("Time (s)");
    ui->plot_x2->xAxis->setLabel("Time (s)");
    ui->plot_x1->yAxis->setLabel("X Coordinates");
    ui->plot_x2->yAxis->setLabel("X Coordinates");
    ui->plot_y1->xAxis->setLabel("Time (s)");
    ui->plot_y2->xAxis->setLabel("Time (s)");
    ui->plot_y1->yAxis->setLabel("Y Coordinates");
    ui->plot_y2->yAxis->setLabel("Y Coordinates");
}

void Telemetry::enable_graph_legends(bool enable)
{
    // Enable graph legends.
    ui->plot_height1->legend->setVisible(enable);
    ui->plot_height2->legend->setVisible(enable);
    ui->plot_yaw1->legend->setVisible(enable);
    ui->plot_yaw2->legend->setVisible(enable);
    ui->plot_x1->legend->setVisible(enable);
    ui->plot_x2->legend->setVisible(enable);
    ui->plot_y1->legend->setVisible(enable);
    ui->plot_y2->legend->setVisible(enable);
}

void Telemetry::set_interactions_settings(bool drag, bool zoom)
{
    // Set interaction setting for graph dragging.
    ui->plot_height1->setInteraction(QCP::iRangeDrag, drag);
    ui->plot_height2->setInteraction(QCP::iRangeDrag, drag);
    ui->plot_yaw1->setInteraction(QCP::iRangeDrag, drag);
    ui->plot_yaw2->setInteraction(QCP::iRangeDrag, drag);
    ui->plot_x1->setInteraction(QCP::iRangeDrag, drag);
    ui->plot_x2->setInteraction(QCP::iRangeDrag, drag);
    ui->plot_y1->setInteraction(QCP::iRangeDrag, drag);
    ui->plot_y2->setInteraction(QCP::iRangeDrag, drag);

    // Set interaction setting for graph zooming.
    ui->plot_height1->setInteraction(QCP::iRangeZoom, zoom);
    ui->plot_height2->setInteraction(QCP::iRangeZoom, zoom);
    ui->plot_yaw1->setInteraction(QCP::iRangeZoom, zoom);
    ui->plot_yaw2->setInteraction(QCP::iRangeZoom, zoom);
    ui->plot_x1->setInteraction(QCP::iRangeZoom, zoom);
    ui->plot_x2->setInteraction(QCP::iRangeZoom, zoom);
    ui->plot_y1->setInteraction(QCP::iRangeZoom, zoom);
    ui->plot_y2->setInteraction(QCP::iRangeZoom, zoom);
}

void Telemetry::init_yValues()
{
    // Initialize y values to 0.
    yThroTarget1.push_back(0.);
    yThroTarget2.push_back(0.);
    yYawTarget1.push_back(0.);
    yYawTarget2.push_back(0.);
    yCoordinatesTar1.push_back(0.);
    yCoordinatesTar2.push_back(0.);
}
/// QUAD 0
void Telemetry::updatePTermPitch0(const merryMsg::Msg::ConstPtr &pTerm)
{
    yPTermPitch[0].push_back((double)pTerm.get()->data);
    xPTimePitch[0].push_back((double)pTerm.get()->header.stamp.toSec() - zero_reference);

    //std::cerr << yPTermPitch[0].last() << std::endl;
    ui->plot_x1->graph(0)->setData(xPTimePitch[0], yPTermPitch[0]);
    ui->plot_x1->xAxis->setRange(xPTimePitch[0].first(), xPTimePitch[0].last());
    ui->plot_x1->replot();
}

void Telemetry::updateITermPitch0(const merryMsg::Msg::ConstPtr &iTerm)
{
    yITermPitch[0].push_back(iTerm.get()->data);
    xITimePitch[0].push_back(iTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_x1->graph(1)->setData(xPTimePitch[0], yITermPitch[0]);
    ui->plot_x1->xAxis->setRange(xPTimePitch[0].first(), xITimePitch[0].last());
    ui->plot_x1->replot();
}

void Telemetry::updateDTermPitch0(const merryMsg::Msg::ConstPtr &dTerm)
{
     yDTermPitch[0].push_back(dTerm.get()->data);
     xDTimePitch[0].push_back(dTerm.get()->header.stamp.toSec() - zero_reference);

     ui->plot_x1->graph(2)->setData(xPTimePitch[0], yDTermPitch[0]);
     ui->plot_x1->xAxis->setRange(xPTimePitch[0].first(), xDTimePitch[0].last());
     ui->plot_x1->replot();
}

void Telemetry::updateOutputPitch0(const merryMsg::Msg::ConstPtr &output)
{
    yOutputPitch[0].push_back(output.get()->data);
    xOutputPitch[0].push_back(output.get()->header.stamp.toSec() - zero_reference);

    ui->plot_x1->graph(3)->setData(xPTimePitch[0], yOutputPitch[0]);
    ui->plot_x1->xAxis->setRange(xPTimePitch[0].first(), xOutputPitch[0].last());
    ui->plot_x1->replot();
}

void Telemetry::updatePTermRoll0(const merryMsg::Msg::ConstPtr &pTerm)
{
    yPTermRoll[0].push_back(pTerm.get()->data);
    xPTimeRoll[0].push_back(pTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y1->graph(0)->setData(xPTimeRoll[0], yPTermRoll[0]);
    ui->plot_y1->xAxis->setRange(xPTimeRoll[0].first(), xPTimeRoll[0].last());
    ui->plot_y1->replot();
}

void Telemetry::updateITermRoll0(const merryMsg::Msg::ConstPtr &iTerm)
{
    yITermRoll[0].push_back(iTerm.get()->data);
    xITimeRoll[0].push_back(iTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y1->graph(1)->setData(xITimeRoll[0], yITermRoll[0]);
    ui->plot_y1->xAxis->setRange(xITimeRoll[0].first(), xITimeRoll[0].last());
    ui->plot_y1->replot();
}

void Telemetry::updateDTermRoll0(const merryMsg::Msg::ConstPtr &dTerm)
{
    yDTermRoll[0].push_back(dTerm.get()->data);
    xDTimeRoll[0].push_back(dTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y1->graph(2)->setData(xDTimeRoll[0], yDTermRoll[0]);
    ui->plot_y1->xAxis->setRange(xDTimeRoll[0].first(), xDTimeRoll[0].last());
    ui->plot_y1->replot();
}

void Telemetry::updateOutputRoll0(const merryMsg::Msg::ConstPtr &output)
{
    yOutputRoll[0].push_back(output.get()->data);
    xOutputRoll[0].push_back(output.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y1->graph(3)->setData(xOutputRoll[0], yOutputRoll[0]);
    ui->plot_y1->xAxis->setRange(xOutputRoll[0].first(), xOutputRoll[0].last());
    ui->plot_y1->replot();
}

/// QUAD 1

void Telemetry::updatePTermPitch1(const merryMsg::Msg::ConstPtr &pTerm)
{
    yPTermPitch[1].push_back(pTerm.get()->data);
    xPTimePitch[1].push_back(pTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_x2->graph(1)->setData(xPTimePitch[1], yPTermPitch[1]);
    ui->plot_x2->xAxis->setRange(xPTimePitch[1].first(), xPTimePitch[1].last());
    ui->plot_x2->replot();
}

void Telemetry::updateITermPitch1(const merryMsg::Msg::ConstPtr &iTerm)
{
    yITermPitch[1].push_back(iTerm.get()->data);
    xITimePitch[1].push_back(iTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_x2->graph(1)->setData(xITimePitch[1], yITermPitch[1]);
    ui->plot_x2->xAxis->setRange(xITimePitch[1].first(), xITimePitch[1].last());
    ui->plot_x2->replot();
}

void Telemetry::updateDTermPitch1(const merryMsg::Msg::ConstPtr &dTerm)
{
     yDTermPitch[1].push_back(dTerm.get()->data);
     xDTimePitch[1].push_back(dTerm.get()->header.stamp.toSec() - zero_reference);

     ui->plot_x2->graph(2)->setData(xDTimePitch[1], yDTermPitch[1]);
     ui->plot_x2->xAxis->setRange(xDTimePitch[1].first(), xDTimePitch[1].last());
     ui->plot_x2->replot();
}

void Telemetry::updateOutputPitch1(const merryMsg::Msg::ConstPtr &output)
{
    yOutputPitch[1].push_back(output.get()->data);
    xOutputPitch[1].push_back(output.get()->header.stamp.toSec() - zero_reference);

    ui->plot_x2->graph(3)->setData(xOutputPitch[1], yOutputPitch[1]);
    ui->plot_x2->xAxis->setRange(xOutputPitch[1].first(), xOutputPitch[1].last());
    ui->plot_x2->replot();
}

void Telemetry::updatePTermRoll1(const merryMsg::Msg::ConstPtr &pTerm)
{
    yPTermRoll[1].push_back(pTerm.get()->data);
    xPTimeRoll[1].push_back(pTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y2->graph(1)->setData(xPTimeRoll[1], yPTermRoll[1]);
    ui->plot_y2->xAxis->setRange(xPTimeRoll[1].first(), xPTimeRoll[1].last());
    ui->plot_y2->replot();
}

void Telemetry::updateITermRoll1(const merryMsg::Msg::ConstPtr &iTerm)
{
    yITermRoll[1].push_back(iTerm.get()->data);
    xITimeRoll[1].push_back(iTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y2->graph(1)->setData(xITimeRoll[1], yITermRoll[1]);
    ui->plot_y2->xAxis->setRange(xITimeRoll[1].first(), xITimeRoll[1].last());
    ui->plot_y2->replot();
}

void Telemetry::updateDTermRoll1(const merryMsg::Msg::ConstPtr &dTerm)
{
    yDTermRoll[1].push_back(dTerm.get()->data);
    xDTimeRoll[1].push_back(dTerm.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y2->graph(2)->setData(xDTimeRoll[1], yDTermRoll[1]);
    ui->plot_y2->xAxis->setRange(xDTimeRoll[1].first(), xDTimeRoll[1].last());
    ui->plot_y2->replot();
}

void Telemetry::updateOutputRoll1(const merryMsg::Msg::ConstPtr &output)
{
    yOutputRoll[1].push_back(output.get()->data);
    xOutputRoll[1].push_back(output.get()->header.stamp.toSec() - zero_reference);

    ui->plot_y2->graph(3)->setData(xOutputRoll[1], yOutputRoll[1]);
    ui->plot_y2->xAxis->setRange(xOutputRoll[1].first(), xOutputRoll[1].last());
    ui->plot_y2->replot();
}
/// OTHER PARTS
void Telemetry::updateThrottleTableActual1(const merryMsg::Msg::ConstPtr& thro)
{
    yThro1.push_back(thro.get()->data);
    xThro1.push_back(thro.get()->header.stamp.toSec() - zero_reference);
    yThroDiff1.push_back(fabs(yThro1.last()-yThroTarget1.last()));

    ui->plot_height1->graph(1)->setData(xThro1, yThro1);
    ui->plot_height1->graph(2)->setData(xThro1, yThroDiff1);

    ui->plot_height1->xAxis->setRange(xThro1.first(), xThro1.last());
    ui->plot_height1->replot();
}

void Telemetry::updateYawTableActual1(const merryMsg::Msg::ConstPtr& yaw)
{
    yYaw1.push_back(yaw.get()->data);
    xYaw1.push_back(yaw.get()->header.stamp.toSec() - zero_reference);
    yYawDiff1.push_back(fabs(yYaw1.last()-yYawTarget1.last()));

    ui->plot_yaw1->graph(1)->setData(xYaw1, yYaw1);
    ui->plot_yaw1->graph(2)->setData(xYaw1, yYawDiff1);

    ui->plot_yaw1->xAxis->setRange(xYaw1.first(), xYaw1.last());
    ui->plot_yaw1->replot();
}

void Telemetry::updateThrottleTableActual2(const merryMsg::Msg::ConstPtr& thro)
{
    yThro2.push_back(thro.get()->data);
    xThro2.push_back(thro.get()->header.stamp.toSec() - zero_reference);
    yThroDiff2.push_back(fabs(yThro2.last()-yThroTarget2.last()));

    ui->plot_height2->graph(1)->setData(xThro2, yThro2);
    ui->plot_height2->graph(2)->setData(xThro2, yThroDiff2);

    ui->plot_height2->xAxis->setRange(xThro2.first(), xThro2.last());
    ui->plot_height2->replot();
}

void Telemetry::updateYawTableActual2(const merryMsg::Msg::ConstPtr& yaw)
{
    yYaw2.push_back(yaw.get()->data);
    xYaw2.push_back(yaw.get()->header.stamp.toSec() - zero_reference);
    yYawDiff2.push_back(fabs(yYaw2.last()-yYawTarget2.last()));

    ui->plot_yaw2->graph(1)->setData(xYaw2, yYaw2);
    ui->plot_yaw2->graph(2)->setData(xYaw2, yYawDiff2);

    ui->plot_yaw2->xAxis->setRange(xYaw2.first(), xYaw2.last());
    ui->plot_yaw2->replot();
}

void Telemetry::updateThrottleTableDesired1(const merryMsg::Msg::ConstPtr& thro)
{
    yThroTarget1.push_back(thro.get()->data);
    xThroTarget1.push_back(thro.get()->header.stamp.toSec() - zero_reference);
    ui->plot_height1->graph(0)->setData(xThroTarget1, yThroTarget1);
    ui->plot_height1->replot();
}

void Telemetry::updateYawTableDesired1(const merryMsg::Msg::ConstPtr& yaw)
{
    /*
    yaw_tarTemp1 = yaw_tarTemp1 + yaw.get()->data;
    if(yaw_tarTemp1 > 180.)
    {
        yaw_tarTemp1 = yaw_tarTemp1 - 180.;
        yaw_tarTemp1 = -180. + yaw_tarTemp1;
    }
    else if(yaw_tarTemp1 < -180.)
    {
        yaw_tarTemp1 = yaw_tarTemp1 + 180.;
        yaw_tarTemp1 = 180 + yaw_tarTemp1;
    }
    yYawTarget1.push_back(yaw_tarTemp1);
*/
    yYawTarget1.push_back(yaw.get()->data);

    xYawTarget1.push_back(yaw.get()->header.stamp.toSec() - zero_reference);
    ui->plot_yaw1->graph(0)->setData(xYawTarget1, yYawTarget1);
    ui->plot_yaw1->replot();
}

void Telemetry::updateThrottleTableDesired2(const merryMsg::Msg::ConstPtr& thro)
{
    yThroTarget2.push_back(thro.get()->data);
    xThroTarget2.push_back(thro.get()->header.stamp.toSec() - zero_reference);
    ui->plot_height2->graph(0)->setData(xThroTarget2, yThroTarget2);
    ui->plot_height2->replot();
}

void Telemetry::updateYawTableDesired2(const merryMsg::Msg::ConstPtr& yaw)
{
    /*
    yaw_tarTemp2 = yaw_tarTemp2 + yaw.get()->data;
    if(yaw_tarTemp2 > 180.)
    {
        yaw_tarTemp2 = yaw_tarTemp2 - 180.;
        yaw_tarTemp2 = -180. + yaw_tarTemp2;
    }
    else if(yaw_tarTemp2 < -180.)
    {
        yaw_tarTemp2 = yaw_tarTemp2 + 180.;
        yaw_tarTemp2 = 180 + yaw_tarTemp2;
    }
    yYawTarget2.push_back(yaw_tarTemp2);
*/
    yYawTarget2.push_back(yaw.get()->data);

    xYawTarget2.push_back(yaw.get()->header.stamp.toSec() - zero_reference);
    ui->plot_yaw2->graph(0)->setData(xYawTarget2, yYawTarget2);
    ui->plot_yaw2->replot();
}

void Telemetry::updateCoordinatesTableDesired1(const geometry_msgs::Point& p)
{
    waypoint[0] = p;
}

void Telemetry::updateCoordinatesTableDesired2(const geometry_msgs::Point& p)
{
    waypoint[1] = p;
}

void Telemetry::actualCoordinates()
{
//    static double minCoordinates[2][2] = {0}, maxCoordinates[2][2] = {0};

//    try
//    {
//        listener.lookupTransform("map", "camera0_link", ros::Time(0), transform);
//        xCoordinates1.push_back(transform.getOrigin().x());
//        yCoordinates1.push_back(transform.getOrigin().y());
//    }
//    catch(tf::TransformException ex)
//    {
//        qDebug() << "camera0_link has no transforms available";
//    }

//    try
//    {
//        listener.lookupTransform("map", "camera1_link", ros::Time(0), transform);
//        xCoordinates2.push_back(transform.getOrigin().x());
//        yCoordinates2.push_back(transform.getOrigin().y());
//    }
//    catch(tf::TransformException ex)
//    {
//        qDebug() << "camera1_link has no transforms available";
//    }

//    time1.push_back(ros::Time::now().toSec() - zero_reference);
//    double t = 0;
//    if (time1.size() > 1)
//        t = time1[time1.size()-1] - time1[time1.size()-2];

//    time2.push_back(ros::Time::now().toSec() - zero_reference);
//    time3.push_back(ros::Time::now().toSec() - zero_reference);

//    xCoordinatesTar1.push_back(waypoint[0].x);
//    xCoordinatesTar2.push_back(waypoint[1].x);
//    yCoordinatesTar1.push_back(waypoint[0].y);
//    yCoordinatesTar2.push_back(waypoint[1].y);

//    xCoordinatesP1.push_back(xCoordinatesTar1[xCoordinatesTar1.size()-1] - xCoordinates1[xCoordinates1.size()-1]);
//    xCoordinatesP2.push_back(xCoordinatesTar2[xCoordinatesTar2.size()-1] - xCoordinates2[xCoordinates2.size()-1]);
//    yCoordinatesP1.push_back(yCoordinatesTar1[yCoordinatesTar1.size()-1] - yCoordinates1[yCoordinates1.size()-1]);
//    yCoordinatesP2.push_back(yCoordinatesTar2[yCoordinatesTar2.size()-1] - yCoordinates2[yCoordinates2.size()-1]);

//    if (xCoordinatesP1.size() < 2)
//        xCoordinatesD1.push_back(0);
//    else
//        xCoordinatesD1.push_back((xCoordinatesP1[xCoordinatesP1.size()-1] - xCoordinatesP1[xCoordinatesP1.size()-2])/t);

//    if (yCoordinatesP1.size() < 2)
//        yCoordinatesD1.push_back(0);
//    else
//        yCoordinatesD1.push_back((yCoordinatesP1[yCoordinatesP1.size()-1] - yCoordinatesP1[xCoordinatesP1.size()-2])/t);

//    if (xCoordinatesP2.size() < 2)
//        xCoordinatesD2.push_back(0);
//    else
//        xCoordinatesD2.push_back((xCoordinatesP2[xCoordinatesP2.size()-1] - xCoordinatesP2[xCoordinatesP2.size()-2])/t);

//    if (yCoordinatesP2.size() < 2)
//        yCoordinatesD2.push_back(0);
//    else
//        yCoordinatesD2.push_back((yCoordinatesP2[yCoordinatesP2.size()-1] - yCoordinatesP2[yCoordinatesP2.size()-2])/t);

//    ui->plot_x1->graph(0)->setData(time1, xCoordinatesTar1);
//    ui->plot_y1->graph(0)->setData(time1, yCoordinatesTar1);
//    ui->plot_x2->graph(0)->setData(time1, xCoordinatesTar2);
//    ui->plot_y2->graph(0)->setData(time1, yCoordinatesTar2);

//    ui->plot_x1->graph(1)->setData(time1, xCoordinates1);
//    ui->plot_y1->graph(1)->setData(time1, yCoordinates1);
//    ui->plot_x2->graph(1)->setData(time1, xCoordinates2);
//    ui->plot_y2->graph(1)->setData(time1, yCoordinates2);

//    ui->plot_x1->graph(2)->setData(time1, xCoordinatesP1);
//    ui->plot_y1->graph(2)->setData(time1, yCoordinatesP1);
//    ui->plot_x2->graph(2)->setData(time1, xCoordinatesP2);
//    ui->plot_y2->graph(2)->setData(time1, yCoordinatesP2);

//    ui->plot_x1->graph(3)->setData(time1, xCoordinatesD1);
//    ui->plot_y1->graph(3)->setData(time1, yCoordinatesD1);
//    ui->plot_x2->graph(3)->setData(time1, xCoordinatesD2);
//    ui->plot_y2->graph(3)->setData(time1, yCoordinatesD2);

//    ui->plot_x1->xAxis->setRange(0, time1[time1.size()-1]);
//    ui->plot_x2->xAxis->setRange(0, time1[time1.size()-1]);
//    ui->plot_y1->xAxis->setRange(0, time1[time1.size()-1]);
//    ui->plot_y2->xAxis->setRange(0, time1[time1.size()-1]);

//    if (waypoint[0].x < minCoordinates[0][0])
//        minCoordinates[0][0] = waypoint[0].x;
//    if (waypoint[0].y < minCoordinates[0][1])
//        minCoordinates[0][1] = waypoint[0].y;
//    if (waypoint[1].x < minCoordinates[1][0])
//        minCoordinates[1][0] = waypoint[1].x;
//    if (waypoint[1].y < minCoordinates[1][1])
//        minCoordinates[1][1] = waypoint[1].y;

//    double i = xCoordinates1.size()-1;
//    minCoordinates[0][0] = std::min(std::min(std::min(std::min(xCoordinates1[i], xCoordinatesTar1[i]),xCoordinatesP1[i]),xCoordinatesD1[i]),minCoordinates[0][0]);
//    minCoordinates[0][1] = std::min(std::min(std::min(std::min(yCoordinates2[i], yCoordinatesTar2[i]),yCoordinatesP2[i]),yCoordinatesD2[i]),minCoordinates[0][1]);
//    minCoordinates[1][0] = std::min(std::min(std::min(std::min(xCoordinates1[i], xCoordinatesTar1[i]),xCoordinatesP1[i]),xCoordinatesD1[i]),minCoordinates[1][0]);
//    minCoordinates[1][1] = std::min(std::min(std::min(std::min(yCoordinates2[i], yCoordinatesTar2[i]),yCoordinatesP2[i]),yCoordinatesD2[i]),minCoordinates[1][1]);

//    maxCoordinates[0][0] = std::max(std::max(std::max(std::max(xCoordinates1[i], xCoordinatesTar1[i]),xCoordinatesP1[i]),xCoordinatesD1[i]),maxCoordinates[0][0]);
//    maxCoordinates[0][1] = std::max(std::max(std::max(std::max(yCoordinates2[i], yCoordinatesTar2[i]),yCoordinatesP2[i]),yCoordinatesD2[i]),maxCoordinates[0][1]);
//    maxCoordinates[1][0] = std::max(std::max(std::max(std::max(xCoordinates1[i], xCoordinatesTar1[i]),xCoordinatesP1[i]),xCoordinatesD1[i]),maxCoordinates[1][0]);
//    maxCoordinates[1][1] = std::max(std::max(std::max(std::max(yCoordinates2[i], yCoordinatesTar2[i]),yCoordinatesP2[i]),yCoordinatesD2[i]),maxCoordinates[1][1]);

//    ui->plot_x1->yAxis->setRange(minCoordinates[0][0], maxCoordinates[0][0]);
//    ui->plot_y1->yAxis->setRange(minCoordinates[0][1], maxCoordinates[0][1]);

//    ui->plot_x2->yAxis->setRange(minCoordinates[1][0], maxCoordinates[1][0]);
//    ui->plot_y2->yAxis->setRange(minCoordinates[1][1], maxCoordinates[1][1]);

//    ui->plot_x1->replot();
//    ui->plot_y1->replot();
//    ui->plot_x2->replot();
//    ui->plot_y2->replot();
}

void Telemetry::adjustBatteryLevel1(const merryMsg::Msg::ConstPtr& battery)
{
    ui->pb_quad1->setValue(battery.get()->data);
}

void Telemetry::adjustBatteryLevel2(const merryMsg::Msg::ConstPtr& battery)
{
    ui->pb_quad2->setValue(battery.get()->data);
}

void Telemetry::on_btn_height_clicked()
{
    cHeight.data = ui->txt_height->text().toFloat();
    cHeight.header.stamp = ros::Time::now();
    if(ui->comboBox->currentIndex() == 0)
        pub_height.publish(cHeight);
    else if(ui->comboBox->currentIndex() == 1)
        pub_height2.publish(cHeight);
}

void Telemetry::on_btn_yaw_clicked()
{
    cYaw.data = ui->txt_yaw->text().toFloat();
    cYaw.header.stamp = ros::Time::now();
    if(ui->comboBox->currentIndex() == 0)
        pub_yaw.publish(cYaw);
    else if(ui->comboBox->currentIndex() == 1)
        pub_yaw2.publish(cYaw);
}

void Telemetry::on_btn_start_clicked()
{
    //ui->btn_start->setEnabled(false);
    if(ui->btn_start->text() == "START")
        ui->btn_start->setText("UNSTART");
    else if(ui->btn_start->text() == "UNSTART")
        ui->btn_start->setText("START");
    std_msgs::Bool x;
    x.data = true;
    if(ui->comboBox->currentIndex() == 0)
        pub_start.publish(x);
    else if(ui->comboBox->currentIndex() == 1)
        pub_start2.publish(x);
}

void Telemetry::on_btn_arm_clicked()
{
    if(ui->btn_arm->text() == "ARM")
        ui->btn_arm->setText("DISARM");
    else if(ui->btn_arm->text() == "DISARM")
        ui->btn_arm->setText("ARM");

    std_msgs::Bool x;
    x.data = true;
    if(ui->comboBox->currentIndex() == 0)
        pub_arm.publish(x);
    else if(ui->comboBox->currentIndex() == 1)
        pub_arm2.publish(x);
}

void Telemetry::on_btn_reset_clicked()
{
    this->zero_reference = ros::Time::now().toSec();

    xThro1.clear();
    xThro2.clear();
    xYaw1.clear();
    xYaw2.clear();

    yThro1.clear();
    yThro2.clear();
    yYaw1.clear();
    yYaw2.clear();

    xThroTarget1.clear();
    xThroTarget2.clear();
    xYawTarget1.clear();
    xYawTarget2.clear();

    yThroTarget1.clear();
    yThroTarget2.clear();
    yYawTarget1.clear();
    yYawTarget2.clear();

    xThroDiff1.clear();
    xThroDiff2.clear();
    xYawDiff1.clear();
    xYawDiff2.clear();

    yThroDiff1.clear();
    yThroDiff2.clear();
    yYawDiff1.clear();
    yYawDiff2.clear();

    ui->plot_height1->replot();
    ui->plot_height2->replot();
    ui->plot_yaw1->replot();
    ui->plot_yaw2->replot();
}

void Telemetry::on_btn_print_clicked()
{
    ui->plot_height1->saveJpg("plot_height1");
    ui->plot_height2->saveJpg("plot_height2");
    ui->plot_yaw1->saveJpg("plot_yaw1");
    ui->plot_yaw2->saveJpg("plot_yaw2");
}

void Telemetry::quad1_connected()
{
    ui->lbl_quad1_stat->setText("Connected");
    ui->lbl_quad1_stat->setStyleSheet("QLabel {color:green;}");
}

void Telemetry::quad1_disconnected()
{
    ui->lbl_quad1_stat->setText("Disconnected!");
    ui->lbl_quad1_stat->setStyleSheet("QLabel {color:red;}");
}

void Telemetry::quad2_connected()
{
    ui->lbl_quad2_stat->setText("Connected");
    ui->lbl_quad2_stat->setStyleSheet("QLabel {color:green;}");
}

void Telemetry::quad2_disconnected()
{
    ui->lbl_quad2_stat->setText("Disconnected!");
    ui->lbl_quad2_stat->setStyleSheet("QLabel {color:red;}");
}

void Telemetry::on_btn_legends_clicked()
{
    if(ui->btn_legends->text() == "Hide Legends")
    {
        enable_graph_legends(false);
        ui->btn_legends->setText("Show Legends");
    }
    else if (ui->btn_legends->text() == "Show Legends")
    {
        enable_graph_legends(true);
        ui->btn_legends->setText("Hide Legends");
    }
    ui->plot_height1->replot();
    ui->plot_height2->replot();
    ui->plot_yaw1->replot();
    ui->plot_yaw2->replot();
}

void Telemetry::on_btn_start1_clicked()
{
    if(ui->btn_start1->text() == "START")
        ui->btn_start1->setText("UNSTART");
    else if(ui->btn_start1->text() == "UNSTART")
        ui->btn_start1->setText("START");
    std_msgs::Bool x;
    x.data = true;
    pub_start.publish(x);

}

void Telemetry::on_btn_arm1_clicked()
{
    if(ui->btn_arm1->text() == "ARM")
        ui->btn_arm1->setText("DISARM");
    else if(ui->btn_arm1->text() == "DISARM")
        ui->btn_arm1->setText("ARM");

    std_msgs::Bool x;
    x.data = true;
    pub_arm.publish(x);
}

void Telemetry::on_btn_land1_clicked()
{
    std_msgs::Bool x;
    x.data = true;
    pub_land1.publish(x);
    /*
    ros::Rate r(2.5);
    ui->btn_land1->setEnabled(false);
    while(yThroTarget1.last() > -2.)
    {
        ui->lbl_quad1_stat->setText("Landing");
        yThroTarget1.push_back(yThroTarget1.last() -= 0.1);
        cHeight.data = yThroTarget1.last();
        cHeight.header.stamp = ros::Time::now();
        pub_height.publish(cHeight);
        r.sleep();
    }
    ui->btn_arm1->setText("ARM");
    ui->btn_land1->setEnabled(true);
    std_msgs::Bool x;
    x.data = true;
    pub_arm.publish(x);
    */
}

void Telemetry::on_btn_start2_clicked()
{
    if(ui->btn_start2->text() == "START")
        ui->btn_start2->setText("UNSTART");
    else if(ui->btn_start2->text() == "UNSTART")
        ui->btn_start2->setText("START");
    std_msgs::Bool x;
    x.data = true;
    pub_start2.publish(x);
}

void Telemetry::on_btn_arm2_clicked()
{
    if(ui->btn_arm2->text() == "ARM")
        ui->btn_arm2->setText("DISARM");
    else if(ui->btn_arm2->text() == "DISARM")
        ui->btn_arm2->setText("ARM");

    std_msgs::Bool x;
    x.data = true;
    pub_arm2.publish(x);
}

void Telemetry::on_btn_land2_clicked()
{
    std_msgs::Bool x;
    x.data = true;
    pub_land2.publish(x);
    /*
    ros::Rate r(2.5);
    ui->btn_land2->setEnabled(false);
    while(yThroTarget2.last() > -2.)
    {
        ui->lbl_quad2_stat->setText("Landing");
        yThroTarget2.push_back(yThroTarget2.last() -= 0.1);
        cHeight2.data = yThroTarget2.last();
        cHeight2.header.stamp = ros::Time::now();
        pub_height2.publish(cHeight2);
        r.sleep();
    }
    ui->btn_arm2->setText("ARM");
    ui->btn_land2->setEnabled(true);
    std_msgs::Bool x;
    x.data = true;
    pub_arm2.publish(x);
    */
}
