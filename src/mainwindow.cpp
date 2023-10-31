#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QVBoxLayout>
#include <QPushButton>

#include "QDateTime"
#include <QtWidgets>
#include <QString>

#include "QtButtonSlide.h"

#ifndef WIN32
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#endif

// #include "osal/timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>

// osal_timer tt;

// #include "osal.h"
using namespace std;

// #include "EcatDeviceSetup.h"

// #include "log.h"

#ifdef WIN32
//#define     BUILD_WIN           1
#else
//#define     BUILD_LINUX         1
//#define     BUILD_XENO          1
#endif
/**
 * for Xenomai
 */
#if (BUILD_XENO)
#include <alchemy/task.h>
#include <alchemy/timer.h>
#endif

// static void ecatthread(void *arg);
// static void ecatcheck(void *arg);

/**
 * for SOEM Ethercat multiple master
 */
#define EC_VER2
// #include "ethercat.h"
// wget ftp://xmlsoft.org/libxml2/libxml2-2.7.7.tar.gz

#define EC_TIMEOUTMON 500

#if (BUILD_WIN)
char IOmap[4096];
OSAL_THREAD_HANDLE thread_ecatcheck;
OSAL_THREAD_HANDLE thread_ecat_task;
int expectedWKC;
boolean needlf;
volatile int wkc;
volatile int rtcnt;
#else

#endif

const char *if_name = nullptr;

#if (BUILD_WIN)
const char *if_namePino = "\\Device\\NPF_{5F0249A9-EE95-4700-B466-929213790774}"; // PinoHome
const char *if_nameTC = "\\Device\\NPF_{73C70886-3EF3-4CEB-87B1-F78CF253F0AA}";   // TC
// const char* if_name = "\\Device\\NPF_{5F0249A9-EE95-4700-B466-929213790774}";     // HP
// const char* if_name = "\\Device\\NPF_{FD55E04A-135C-4EB0-B6CE-90CB1EE0EE31}";     // Pino
// const char* if_name = "\\Device\\NPF_{70A8B503-D5A5-4595-A10C-CE49126439C6}";     // A301
#elif (BUILD_XENO)
static RT_TASK rtthread_id;
const char *rtthread_name = "RT Thread";

const char *if_name = "rteth0";
// const char* if_name = "eth3";
static pthread_t osthread_id;
const char *osthread_name = "OS Thread";
#elif (BUILD_LINUX)
const char *if_name = "enp3s0"; // MiniBox PC
static pthread_t osthread_id;
const char *osthread_name = "OS Thread";
#endif

static int rt_prio = 60;
static int rt_stacksize = 0; // default stack size

static int stop_thread = 1;

static uint8_t  currentgroup = 0;

/**
 * for Distributed Clock
 */
//#define _USE_DC

/**
 * for Loop test
 */
#define NSEC_PER_SEC 1000000000
#define CLOCK_RES_ns 1e-9 // Clock resolution is 1 ns by default

#define LOOP_PERIOD_10Hz 1e8  // 100msec = 10Hz
#define LOOP_PERIOD_100Hz 1e7 // 10msec = 100Hz
#define LOOP_PERIOD_200Hz 5e6 // 5msec = 200Hz
#define LOOP_PERIOD_250Hz 4e6 // 4msec = 250Hz
#define LOOP_PERIOD_500Hz 2e6 // 2msec = 500Hz

#define LOOP_PERIOD_1KHz 1e6 // 1msec = 1KHz
#define LOOP_PERIOD_2KHz 5e5 // 0.5msec = 2KHz
//#define LOOP_PERIOD_2KHz 500600 //0.5msec = 2KHz
#define LOOP_PERIOD_4KHz 2.5e5  // 0.25msec = 4KHz
#define LOOP_PERIOD_8KHz 1.25e5 // 0.125msec = 8KHz
#define LOOP_PERIOD_10KHz 1e5   // 0.100msec = 10KHz
#define LOOP_PERIOD_20KHz 0.5e5 // 0.050msec = 20KHz

#define DEF_CYCLE_TIME LOOP_PERIOD_500Hz

//#define DEF_LOOP_STOP   1000000000
#define DEF_LOOP_STOP 99999999999999
#define DEF_NUM (unsigned long)(1000 * (1e6 / DEF_CYCLE_TIME) - 1)

#define MAIN_CONTROl_HEIGHT 80
#define LOG_MSG_HEIGHT (100 - MAIN_CONTROl_HEIGHT)

// CEcatMaster* g_ecat = nullptr;

// CEcatSoem _ecatSoem;

#define DEF_WINDOW_FONT    "Arial"
// #define DEF_WINDOW_FONT    "Verdana"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    chart = nullptr;
    timerInit = nullptr;
    timer = nullptr;
    cycleTime = 0;
    exeTime = 0;

    ui->setupUi(this);

    uint8_t fontMainTitleSize = 13;
    uint8_t fontTabTitleSize = 10;
    uint8_t fontGroupTitleSize = 10;
    uint8_t fontLogSize = 8;
    uint8_t fontTextSize = 8;

    QFont fontMainTitle = this->font();
    fontMainTitle.setStyleHint(QFont::Fantasy);
    fontMainTitle.setFamily(DEF_WINDOW_FONT);
    fontMainTitle.setPointSize(fontMainTitleSize);
    fontMainTitle.setWeight(QFont::Bold);

    QFont fontTabTitle = this->font();
    fontTabTitle.setStyleHint(QFont::Fantasy);
    fontTabTitle.setFamily(DEF_WINDOW_FONT);
    fontTabTitle.setPointSize(fontTabTitleSize);
    fontTabTitle.setWeight(QFont::Bold);

    QFont fontGroupTitle = this->font();
    fontGroupTitle.setStyleHint(QFont::Fantasy);
    fontGroupTitle.setFamily(DEF_WINDOW_FONT);
    fontGroupTitle.setPointSize(fontGroupTitleSize);
    fontGroupTitle.setWeight(QFont::Bold);

    QFont fontLog = this->font();
    fontLog.setStyleHint(QFont::Fantasy);
    fontLog.setFamily(DEF_WINDOW_FONT);
    fontLog.setPointSize(fontLogSize);
    fontLog.setWeight(QFont::Bold);

    QFont fontText = this->font();
    fontText.setStyleHint(QFont::Fantasy);
    fontText.setFamily(DEF_WINDOW_FONT);
    fontText.setPointSize(fontTextSize);
    fontText.setWeight(QFont::Bold);

    QColor brown(255, 191, 0);
    QColor ligtGrey(190, 190, 190);
    QColor black(0, 0, 0);
    QColor white(255, 255, 255);
    QPalette palette;
    // white text
    QBrush brush(brown);
    brush.setStyle(Qt::SolidPattern);
    // white text
    QBrush brushW(white);
    brushW.setStyle(Qt::SolidPattern);
    // set white text   
    palette.setBrush(QPalette::Active, QPalette::WindowText, brushW);
    palette.setBrush(QPalette::Inactive, QPalette::WindowText, brushW);

    QPalette paletteW;
    // set white text   
    paletteW.setBrush(QPalette::Active, QPalette::WindowText, brushW);
    paletteW.setBrush(QPalette::Inactive, QPalette::WindowText, brushW);

    // START: CANopen Control/State/Slave Layout---------------------
    // TAB: CANopen Master Control Button and Layout
    QButtonSlide *btnRtnet = new QButtonSlide("RTnet", this->font(), false);
    connect(btnRtnet, SIGNAL(clicked()), SLOT(OnClickedButtonRTnet()));
    QPushButton *btnGetAdapter = new QPushButton("Get NIC");
    connect(btnGetAdapter, SIGNAL(clicked()), SLOT(OnClickedButtonGetAdapter()));
    QPushButton *btnConnect = new QPushButton("CONNECT");
    connect(btnConnect, SIGNAL(clicked()), SLOT(OnClickedButtonConnect()));
    QPushButton *btnStart = new QPushButton("START");
    connect(btnStart, SIGNAL(clicked()), SLOT(OnClickedButtonStart()));
    QPushButton *btnStop = new QPushButton("STOP");
    connect(btnStop, SIGNAL(clicked()), SLOT(OnClickedButtonStop()));
    QPushButton *btnGetSlaveInfo = new QPushButton("SLAVE Info.");
    connect(btnGetSlaveInfo, SIGNAL(clicked()), SLOT(OnClickedButtonGetSlaveInfo()));
    QPushButton *btnLoopTest = new QPushButton("LOOP TEST");

    QGridLayout *gl_EcatControl = new QGridLayout(this);
    uint8_t col = 1;
    gl_EcatControl->addWidget(btnRtnet, 1, col++);
    gl_EcatControl->addWidget(btnGetAdapter, 1, col++);
    gl_EcatControl->addWidget(btnConnect, 1, col++);
    gl_EcatControl->addWidget(btnStart, 1, col++);
    gl_EcatControl->addWidget(btnStop, 1, col++);
    gl_EcatControl->addWidget(btnGetSlaveInfo, 1, col++);
    gl_EcatControl->addWidget(btnLoopTest, 1, col++);

    lineEditCycleTime = new QLineEdit("0");
    lineEditCycleTime->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    lineEditCycleTime->setMinimumWidth(70);
    lineEditexeTime = new QLineEdit("0");
    lineEditexeTime->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    lineEditexeTime->setMinimumWidth(70);
    QGridLayout *gl_EcatControlTime = new QGridLayout(this);
    col = 1;
    gl_EcatControlTime->addWidget(new QLabel("Cycle Time [usec]:"), 1, col++);
    gl_EcatControlTime->addWidget(lineEditCycleTime, 1, col++);
    gl_EcatControlTime->addWidget(new QLabel("Execution Time [usec]:"), 1, col++);
    gl_EcatControlTime->addWidget(lineEditexeTime, 1, col++);

    QVBoxLayout *vb_EcatControl = new QVBoxLayout(this);
    vb_EcatControl->addLayout(gl_EcatControl);
    vb_EcatControl->addLayout(gl_EcatControlTime);
    QWidget *wg_EcatControl = new QWidget();
    wg_EcatControl->setLayout(vb_EcatControl);
    wg_EcatControl->setFont(fontText);

    QTabWidget *tab_EcatControl = new QTabWidget();
    tab_EcatControl->addTab(wg_EcatControl, QIcon("app.ico"), QString("CANopen Master Control"));
    tab_EcatControl->setFont(fontTabTitle);
    tab_EcatControl->setPalette(palette);    
    //-------------------------------------------------------------

    // TAB: CANopen State
    ecatStateBtn.ledEcatState = LedLayout("Current State:", "INIT", LED_GREEN);
    ecatStateBtn.btnStInit = new QPushButton("INIT");
    connect(ecatStateBtn.btnStInit, SIGNAL(clicked()), SLOT(OnClickedButtonEcatStToInit()));
    ecatStateBtn.btnStPreOp = new QPushButton("PREOP");
    connect(ecatStateBtn.btnStPreOp, SIGNAL(clicked()), SLOT(OnClickedButtonEcatStToPreOp()));
    ecatStateBtn.btnStSafeOP = new QPushButton("SAFEOP");
    connect(ecatStateBtn.btnStSafeOP, SIGNAL(clicked()), SLOT(OnClickedButtonEcatStToSafeOp()));
    ecatStateBtn.btnStOp = new QPushButton("OP");
    connect(ecatStateBtn.btnStOp, SIGNAL(clicked()), SLOT(OnClickedButtonEcatStToOp()));

    QGridLayout *gl_EcatState = new QGridLayout(this);
    col = 1;
    gl_EcatState->addWidget(ecatStateBtn.btnStInit, 1, col++);
    gl_EcatState->addWidget(ecatStateBtn.btnStPreOp, 1, col++);
    gl_EcatState->addWidget(ecatStateBtn.btnStSafeOP, 1, col++);
    gl_EcatState->addWidget(ecatStateBtn.btnStOp, 1, col++);

    QVBoxLayout *vl_EcatState = new QVBoxLayout(this);
    vl_EcatState->addLayout(ecatStateBtn.ledEcatState.HBox);
    vl_EcatState->addLayout(gl_EcatState);    
    QWidget *wg_EcatSt = new QWidget();
    wg_EcatSt->setLayout(vl_EcatState);
    wg_EcatSt->setFont(fontText);

    QTabWidget *tab_EcatState = new QTabWidget();
    tab_EcatState->addTab(wg_EcatSt, QIcon("app.ico"), QString("CANopen State"));
    tab_EcatState->setFont(fontTabTitle);
    tab_EcatState->setPalette(palette);
    //-------------------------------------------------------------

    // TAB: CANopen Slave Lists
    // tree_SlaveList: member variable
    tree_SlaveList = new QTreeWidget(this);
    tree_SlaveList->setColumnCount(3);
    tree_SlaveList->setColumnWidth(0, 150);
    QStringList headers = {"State", "Device", "Description"};
    tree_SlaveList->setHeaderLabels(headers);

    //-------------------------------------------------------------
    QGridLayout *gl_Ecat = new QGridLayout(this);
    uint8_t row = 1;
    gl_Ecat->addWidget(tab_EcatControl, row++, 1);
    gl_Ecat->addWidget(tab_EcatState, row++, 1);
    gl_Ecat->addWidget(tree_SlaveList, row++, 1);
    gl_Ecat->addWidget(new QLabel(), row++, 1);
    tab_EcatControl->setMinimumHeight(100);
    tab_EcatControl->setMaximumHeight(150);
    tab_EcatState->setMaximumHeight(150);
    // gl_Ecat->setSpacing(100);

    QWidget *gl_EcatWidget = new QWidget();
    gl_EcatWidget->setLayout(gl_Ecat);
    // END: CANopen Control/State/Slave Layout---------------------

   
    // START: Manipulator Servo/Joint/Task Control Tab. Layout -----
    // TAB: Servo Control
    // Servo mode/target/actual pos/vel, move/jog button 생성
    Servo_t header = CreateJointServoLayout(true);
    for (int i = 0; i < DEF_MAX_SERVO; i++)
        jtServo[i] = CreateJointServoLayout();
    // Joint Servo Control Button
    jtServoControl.ledButtonServo = LedButtonLayout("SERVO", "OFF", LED_GREEN, true);
    connect(jtServoControl.ledButtonServo.button, SIGNAL(clicked()), SLOT(OnClickedButtonjtServoOn()));
    jtServoControl.Halt = new QPushButton("HALT");
    jtServoControl.Halt->setMinimumHeight(80);
    connect(jtServoControl.Halt, SIGNAL(clicked()), SLOT(OnClickedButtonJointHalt()));
    jtServoControl.QStop = new QPushButton("QSTOP");
    jtServoControl.QStop->setMinimumHeight(80);
    connect(jtServoControl.QStop, SIGNAL(clicked()), SLOT(OnClickedButtonJointQStop()));
    jtServoControl.Move = new QPushButton("MOVE");
    jtServoControl.Move->setMinimumHeight(80);
    connect(jtServoControl.Move, SIGNAL(clicked()), SLOT(OnClickedButtonJointMove()));
    jtServoControl.GoHome = new QPushButton("HOME");
    jtServoControl.GoHome->setMinimumHeight(80);
    connect(jtServoControl.GoHome, SIGNAL(clicked()), SLOT(OnClickedButtonJointHome()));
    jtServoControl.ResetFault = new QPushButton("RESET");
    jtServoControl.ResetFault->setMinimumHeight(80);
    connect(jtServoControl.ResetFault, SIGNAL(clicked()), SLOT(OnClickedButtonJointReset()));

    // all joints servo control button 생성
    QHBoxLayout *hl_jtServoControl = new QHBoxLayout(this);
    hl_jtServoControl->addLayout(jtServoControl.ledButtonServo.HBox, 25);
    hl_jtServoControl->addWidget(jtServoControl.Halt, 15);
    hl_jtServoControl->addWidget(jtServoControl.QStop, 15);
    hl_jtServoControl->addWidget(jtServoControl.Move, 15);
    hl_jtServoControl->addWidget(jtServoControl.GoHome, 15);
    hl_jtServoControl->addWidget(jtServoControl.ResetFault, 15);

    QVBoxLayout *vb_jtServo = new QVBoxLayout(this);
    vb_jtServo->addLayout(header.HBox);
    for (int i = 0; i < DEF_MAX_SERVO; i++)
        vb_jtServo->addLayout(jtServo[i].HBox);
    vb_jtServo->addLayout(hl_jtServoControl);

    QWidget *tab_Servo = new QWidget();
    tab_Servo->setLayout(vb_jtServo);
    //-------------------------------------------------------------


    // TAB: Joint Control
    QVBoxLayout *vb_TabJoint = new QVBoxLayout(this);
    vb_TabJoint->addWidget(new QLabel("Joint Control"));

    QWidget *tab_Joint = new QWidget();
    tab_Joint->setLayout(vb_TabJoint);
    //-------------------------------------------------------------

    // TAB: Task Control       
    QLabel *labelSetTargetPos = new QLabel("Position [mm]");
    QLineEdit *editSetTargetPos= new QLineEdit("98");
    // editSetTargetPos->setMinimumHeight(lineTextMinHeight);
    editSetTargetPos->setMaximumHeight(100);
    editSetTargetPos->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    editSetTargetPos->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    QPushButton *btnSetTargetPos = new QPushButton("APPLY");
    // btnSetTargetPos->setMinimumHeight(lineTextMinHeight);
    btnSetTargetPos->setMaximumHeight(100);
    connect(btnSetTargetPos, SIGNAL(clicked(bool)), SLOT(OnClickButtonSetTargetPos(bool)));

    QGridLayout *gl_Task = new QGridLayout(this);
    gl_Task->addWidget(labelSetTargetPos, 1, 1);
    gl_Task->addWidget(editSetTargetPos, 1, 2);
    gl_Task->addWidget(btnSetTargetPos, 1, 3);

    QWidget *tab_Task = new QWidget();
    tab_Task->setLayout(gl_Task);
    //-------------------------------------------------------------

    // TAB: Sensor Data       
    QLabel *labelGetForceData = new QLabel("Force [N]");
    QLineEdit *editGetForceData= new QLineEdit("0");
    // editGetForceData->setMinimumHeight(lineTextMinHeight);
    editGetForceData->setMaximumHeight(100);
    editGetForceData->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    editGetForceData->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    editGetForceData->setDisabled(true);
    QPushButton *btnGetForceData = new QPushButton("Read");
    // btnGetForceData->setMinimumHeight(lineTextMinHeight);
    btnGetForceData->setMaximumHeight(100);
    connect(btnGetForceData, SIGNAL(clicked(bool)), SLOT(OnClickButtonGetForceData(bool)));

    QLabel *labelGetTorqueData = new QLabel("Torque [Nm]");
    QLineEdit *editGetTorqueData= new QLineEdit("0");
    // editGetTorqueData->setMinimumHeight(lineTextMinHeight);
    editGetTorqueData->setMaximumHeight(100);
    editGetTorqueData->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    editGetTorqueData->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    editGetTorqueData->setDisabled(true);
    QPushButton *btnGetTorqueData = new QPushButton("Read");
    // btnGetTorqueData->setMinimumHeight(lineTextMinHeight);
    btnGetTorqueData->setMaximumHeight(100);
    connect(btnGetTorqueData, SIGNAL(clicked(bool)), SLOT(OnClickButtonGetTorqueData(bool)));

    QGridLayout *gl_Sensor = new QGridLayout(this);
    gl_Sensor->addWidget(labelGetForceData, 1, 1);
    gl_Sensor->addWidget(editGetForceData, 1, 2);
    gl_Sensor->addWidget(btnGetForceData, 1, 3);
    gl_Sensor->addWidget(labelGetTorqueData, 2, 1);
    gl_Sensor->addWidget(editGetTorqueData, 2, 2);
    gl_Sensor->addWidget(btnGetTorqueData, 2, 3);


    QWidget *tab_Sensor = new QWidget();
    tab_Sensor->setLayout(gl_Sensor);
    //-------------------------------------------------------------

    
    QTabWidget *tabWidgetControl = new QTabWidget();
    tabWidgetControl->addTab(tab_Servo, QString("Servo Drive Control"));
    tabWidgetControl->addTab(tab_Joint, QString("Joint Space Control"));
    tabWidgetControl->addTab(tab_Task, QString("Task Space Control"));
    tabWidgetControl->addTab(tab_Sensor, QString("Read Sensor Data"));
    tabWidgetControl->setFont(fontTabTitle);
    tabWidgetControl->setPalette(palette);
    tab_Servo->setFont(fontText);
    tab_Joint->setFont(fontText);
    tab_Task->setFont(fontText);
    tab_Sensor->setFont(fontText);

    QHBoxLayout *HLayoutMainH1 = new QHBoxLayout(this);
    // HLayoutMainH1->addSpacing(1);
    HLayoutMainH1->addWidget(gl_EcatWidget, 20);
    HLayoutMainH1->addWidget(tabWidgetControl, 80);

    // Log msg.
    textEditLogMsg = new QTextEdit(this);
    textEditLogMsg->setFont(fontLog);

    QVBoxLayout *VLayoutMain = new QVBoxLayout(this);
    VLayoutMain->addLayout(HLayoutMainH1, 70);
    VLayoutMain->addWidget(textEditLogMsg, 30);

    ui->centralwidget->setLayout(VLayoutMain);
    
    connect(btnLoopTest, SIGNAL(clicked()), SLOT(OnClickedButtonLoopTest()));

    // g_ecat  = &ecat;

    // connect(&_log, SIGNAL(signal_LogMsg(QString &)), this, SLOT(OnLogMessage(QString &)));

#if (BUILD_WIN)
    char *temp = nullptr;
    std::string computerName;
    temp = getenv("COMPUTERNAME");
    if (temp)   {
        computerName = temp;
        _log.Debug("Computer Name : %s", temp);
        temp = 0;
    }

    if_name = if_nameTC;
    if      (computerName == "WONDAEHEE-M90Q")  if_name = if_nameTC;
    else if (computerName == "PINO-HP")         if_name = if_namePino;

    ecat.SetIfName(if_name); // HP
#else
    // strcpy(ecat.ec_ifname[0], if_name);
    // ecat.ec_ifname = (char *)if_name;
#endif

    //ecat.SlaveInfoDevName();
    //_ecatSoem.GetSlaveName((char *)if_name);
    //for (int i = 1; i <= ec_slavecount; i++)    {
    //    QString str = QString("Slave: %1").arg(QString::number(i).rightJustified(2, '0'));
    //    AddTreeRootNode(str, ec_slave[i].name);
    //}
    //OnClickedButtonConnect();
    //OnClickedButtonEcatStToInit();
    //OnClickedButtonEcatStToPreOp();

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(OnTimer()));
    timer->start(100);

    timerInit = new QTimer(this);
    connect(timerInit, SIGNAL(timeout()), this, SLOT(OnTimerInit()));
    // timerInit->start(1000);
}

MainWindow::~MainWindow()
{
    stop_thread = 1;
    disconnect(timer, SIGNAL(timeout()), this, SLOT(OnTimer()));

    if (timer)
    {
        timer->stop();
        delete timer;
        timer = nullptr;
    }
    if (timerInit)
    {
        timerInit->stop();
        delete timerInit;
        timerInit = nullptr;
    }

    delete ui;
}

void MainWindow::AddTreeRootNode(const QString &value, const QString &desc)
{
    QTreeWidgetItem *treeItem = new QTreeWidgetItem();
    treeItem->setText(1, value);
    treeItem->setText(2, desc);
    treeItem->setTextAlignment(0, Qt::AlignLeft);
    tree_SlaveList->addTopLevelItem(treeItem);
    tree_SlaveList->expandItem(treeItem);
}

void MainWindow::EcatStateButton(uint16_t state)
{
    static int16_t count = 1000 / timer->interval() / 2;

    // if(_ecatSoem.slave_number > 0)          {
    //     ecatStateBtn.btnStInit->setDisabled(false);
    //     ecatStateBtn.btnStPreOp->setDisabled(false);
    //     ecatStateBtn.btnStSafeOP->setDisabled(false);
    //     ecatStateBtn.btnStOp->setDisabled(false);

    //     ecatStateBtn.ledEcatState.ledLabel->setText(ec_state_str[state]);
    //     ecatStateBtn.ledEcatState.led->setChecked(false);

    //     if (state == EC_STATE_INIT)         {
    //         ecatStateBtn.btnStSafeOP->setDisabled(true);
    //         ecatStateBtn.btnStOp->setDisabled(true);
    //     }
    //     else if (state == EC_STATE_PRE_OP)  {
    //         ecatStateBtn.btnStOp->setDisabled(true);
    //     }
    //     else if (state == EC_STATE_SAFE_OP) {
    //         if (--count < 0)  {
    //             count = 1000 / timer->interval() / 2;
    //             bool value = ecatStateBtn.ledEcatState.led->isChecked();
    //             ecatStateBtn.ledEcatState.led->setChecked(!value);
    //         }
    //     }
    //     else if (state == EC_STATE_OPERATIONAL) {
    //         ecatStateBtn.ledEcatState.led->setChecked(true);
    //     }
    //     else  {
    //     }

    //     for (int i = 0; i <= _ecatSoem.ec_master.ec_slavecount - 1; i++)  {
    //         QTreeWidgetItem *item = tree_SlaveList->topLevelItem(i);
    //         if (_ecatSoem.ec_master.init && item)  {
    //             item->setText(0, ec_state_str[_ecatSoem.ec_master.ec_slave[i].state]);
    //         }
    //     }
    // }
}

void MainWindow::OnLogMessage(QString &sz)
{
    // LogMessage(sz);
}

void MainWindow::LogMessage(const char *sz, ...)
{
    int nLength;
    static char arBuf[256] = {
        0,
    };
    va_list arg_list;

    va_start(arg_list, sz);
    nLength = vsprintf(arBuf, sz, arg_list);
    va_end(arg_list);

    QString str = QString("%1").arg(arBuf);

    /*
    va_list arg_list;
    va_start(arg_list, sz);
    QString str;
    str.vsprintf(sz, arg_list);
    va_end(arg_list);
    */
    // LogMessage(str);
}

void MainWindow::LogMessage(QString &sz)
{
    // static uint32_t lines = 0;
    // QString str;
    // if (++lines > 10000) {
    //     textEditLogMsg->clear();
    //     lines = 0;
    // }

    // str = QDateTime::currentDateTime().toString("HH:mm:ss:zzz") + " | " + sz;
    // logMsgBuff.push_back(str);
}

void MainWindow::LogMessagePrint()
{
    
}    

void MainWindow::closeEvent(QCloseEvent *event)
{

}

void MainWindow::resizeEvent(QResizeEvent *)
{
}

#define DEF_NO_MODE_LIST 10
QString strModeList[DEF_NO_MODE_LIST] = {
    {"0.NO_MODE"},
    {"1.PROFILE_POSITION_MODE"},
    {"2.VELOCITY_MODE"},
    {"3.PROFILE_VELOCITY_MODE"},
    {"4.PROFILE_TORQUE_MODE"},
    {"6.HOMING_MODE"},
    {"7.INTERPOLATION_POSITION_MODE"},
    {"8.CYCLIC_SYNC_POSITION_MODE"},
    {"9.CYCLIC_SYNC_VELOCITY_MODE"},
    {"10.CYCLIC_SYNC_TORQUE_MODE"},
};

Servo_t MainWindow::CreateJointServoLayout(bool header)
{
    Servo_t ret;
    int height = 50;

    QFont fontLabel = this->font();
    fontLabel.setStyleHint(QFont::Fantasy);
    // fontLabel.setFamily("Verdana");
    fontLabel.setFamily("Arial");
    if (header)
    {
        fontLabel.setPointSize(8);
    }
    else
    {
        fontLabel.setPointSize(7);
    }

    QHBoxLayout *HBoxLayout = new QHBoxLayout(this);

    QComboBox *comboMode = new QComboBox();
    comboMode->setMinimumHeight(height);
    // comboMode->setMaximumWidth(100);
    comboMode->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    comboMode->setFixedHeight(height);
    comboMode->setFixedWidth(200);
    comboMode->setFont(fontLabel);

    QLineEdit *editTargetPos = new QLineEdit("");
    editTargetPos->setAlignment(Qt::AlignRight);
    editTargetPos->setFixedHeight(height);
    editTargetPos->setMinimumWidth(100);

    QLineEdit *editActualPos = new QLineEdit("");
    editActualPos->setAlignment(Qt::AlignRight);
    editActualPos->setFixedHeight(height);
    editActualPos->setMinimumWidth(100);
    editActualPos->setDisabled(true);

    QLineEdit *editTargetVel = new QLineEdit(" ");
    editTargetVel->setAlignment(Qt::AlignRight);
    editTargetVel->setFixedHeight(height);
    editTargetVel->setMinimumWidth(100);

    QLineEdit *editActualVel = new QLineEdit(" ");
    editActualVel->setAlignment(Qt::AlignRight);
    editActualVel->setFixedHeight(height);
    editActualVel->setMinimumWidth(100);
    editActualVel->setDisabled(true);

    QLineEdit *editTargetTor = new QLineEdit(" ");
    editTargetTor->setAlignment(Qt::AlignRight);
    editTargetTor->setFixedHeight(height);
    editTargetTor->setMinimumWidth(100);

    QLineEdit *editActualTor = new QLineEdit(" ");
    editActualTor->setAlignment(Qt::AlignRight);
    editActualTor->setFixedHeight(height);
    editActualTor->setMinimumWidth(100);
    editActualTor->setDisabled(true);

    QPushButton *btnMove = new QPushButton("MOVE");
    btnMove->setFixedHeight(height);
    QPushButton *btnJogNDir = new QPushButton("Jog -");
    btnJogNDir->setFixedHeight(height);
    QPushButton *btnJogPDir = new QPushButton("+ Jog");
    btnJogPDir->setFixedHeight(height);

    if (header)
    {
        comboMode->addItem("Mode of Operation");
        comboMode->setDisabled(true);
        editTargetPos->setText("Target Position");
        editTargetPos->setDisabled(true);
        editActualPos->setText("Actual Position");
        editTargetVel->setText("Target Velocity");
        editTargetVel->setDisabled(true);
        editActualVel->setText("Actual Velocity");
        editTargetTor->setText("Target Torque");
        editTargetTor->setDisabled(true);
        editActualTor->setText("Actual Torque");

        btnMove->setDisabled(true);
        btnJogNDir->setDisabled(true);
        btnJogPDir->setDisabled(true);
    }
    else
    {
        for (int i = 0; i < DEF_NO_MODE_LIST; i++)
        {
            comboMode->addItem(strModeList[i]);
        }
        editTargetPos->setText("0");
        editActualPos->setText("0");
        editTargetVel->setText("0");
        editActualVel->setText("0");
        editTargetTor->setText("0");
        editActualTor->setText("0");
    }

    HBoxLayout->addWidget(comboMode);
    HBoxLayout->addWidget(editTargetPos);
    HBoxLayout->addWidget(editActualPos);
    HBoxLayout->addWidget(editTargetVel);
    HBoxLayout->addWidget(editActualVel);
    HBoxLayout->addWidget(editTargetTor);
    HBoxLayout->addWidget(editActualTor);

    HBoxLayout->addWidget(btnMove);
    HBoxLayout->addWidget(btnJogNDir);
    HBoxLayout->addWidget(btnJogPDir);

    // ret.labelTargetPos  = labelTargetPos;
    ret.editTargetPos = editTargetPos;
    // ret.labelActualPos  = labelActualPos;
    ret.editActualPos = editActualPos;

    // ret.labelTargetVel  = labelTargetVel;
    ret.editTargetVel = editTargetVel;
    // ret.labelActualVel  = labelActualVel;
    ret.editActualVel = editActualVel;

    // ret.labelTargetTor  = labelTargetTor;
    ret.editTargetTor = editTargetTor;
    // ret.labelActualTor  = labelActualTor;
    ret.editActualTor = editActualTor;

    ret.btnMove = btnMove;
    ret.btnJogNDir = btnJogNDir;
    ret.btnJogPDir = btnJogPDir;

    ret.HBox = HBoxLayout;

    return ret;
}

Led_t MainWindow::LedLayout(QString strLedName, QString strLedLabel, led_color co)
{
    Led_t ret;

    QFont fontLabel = this->font();
    fontLabel.setStyleHint(QFont::Fantasy);
    // fontLabel.setFamily("Verdana");
    fontLabel.setFamily("Arial");

    QHBoxLayout *HBoxLayout = new QHBoxLayout(this);
    QButtonLedIndicator *btnLed = new QButtonLedIndicator(this);
    btnLed->setDisabled(true);
    btnLed->setMinimumHeight(20);
    // btnLed->setMaximumHeight(100);
    if (co == LED_RED)
    {
        btnLed->setOnColor1(QColor(255, 0, 0));
        btnLed->setOnColor2(QColor(176, 0, 0));
        btnLed->setOffColor1(QColor(28, 0, 0));
        btnLed->setOffColor2(QColor(156, 0, 0));
    }
    else if (co == LED_YELLOW)
    {
        btnLed->setOnColor1(QColor(255, 255, 0));
        btnLed->setOnColor2(QColor(176, 176, 0));
        btnLed->setOffColor1(QColor(28, 28, 0));
        btnLed->setOffColor2(QColor(156, 156, 0));
    }
    else
    {
        btnLed->setOnColor1(QColor(0, 255, 0));
        btnLed->setOnColor2(QColor(0, 192, 0));
        btnLed->setOffColor1(QColor(0, 28, 0));
        btnLed->setOffColor2(QColor(0, 128, 0));
    }

    QLabel *ledName = new QLabel(strLedName);
    ledName->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    ledName->setFont(fontLabel);

    QLabel *ledLabel = new QLabel(strLedLabel);
    ledLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    ledLabel->setFont(fontLabel);
    ledLabel->setFixedWidth(100);

    QLabel *label = new QLabel("");

    HBoxLayout->addWidget(label, 65);
    HBoxLayout->addWidget(ledName, 20);
    HBoxLayout->addWidget(ledLabel, 10);
    HBoxLayout->addWidget(btnLed, 5);

    ret.led = btnLed;
    ret.ledLabel = ledLabel;
    ret.HBox = HBoxLayout;
    return ret;
}

LedButton_t MainWindow::LedButtonLayout(QString strBtnName, QString strLedName, led_color co, bool btnVisible)
{
    QFont fontLabel = this->font();
    fontLabel.setStyleHint(QFont::Fantasy);
    // fontLabel.setFamily("Verdana");
    fontLabel.setFamily("Arial");
    // fontLabel.setPointSize(7);
    // fontLabel.setWeight(QFont::Bold);
    QRect screenres = QApplication::desktop()->screenGeometry(1 /*screenNumber*/);
    if (screenres.width() <= 1920)
    {
        fontLabel.setPointSize(10);
    }
    else
    {
        fontLabel.setPointSize(10);
    }

    LedButton_t ret;

    QHBoxLayout *HBoxLayout = new QHBoxLayout(this);
    QVBoxLayout *VBoxLayout = new QVBoxLayout(this);
    QPushButton *btn = new QPushButton(strBtnName);
    btn->setMinimumHeight(30);
    btn->setMaximumHeight(200);
    btn->setVisible(btnVisible);
    QButtonLedIndicator *btnLed = new QButtonLedIndicator(this);
    btnLed->setDisabled(true);
    btnLed->setMinimumHeight(20);
    btnLed->setMaximumHeight(100);
    if (co == LED_RED)
    {
        btnLed->setOnColor1(QColor(255, 0, 0));
        btnLed->setOnColor2(QColor(176, 0, 0));
        btnLed->setOffColor1(QColor(28, 0, 0));
        btnLed->setOffColor2(QColor(156, 0, 0));
    }
    else if (co == LED_YELLOW)
    {
        btnLed->setOnColor1(QColor(255, 255, 0));
        btnLed->setOnColor2(QColor(176, 176, 0));
        btnLed->setOffColor1(QColor(28, 28, 0));
        btnLed->setOffColor2(QColor(156, 156, 0));
    }
    else
    {
        btnLed->setOnColor1(QColor(0, 255, 0));
        btnLed->setOnColor2(QColor(0, 192, 0));
        btnLed->setOffColor1(QColor(0, 28, 0));
        btnLed->setOffColor2(QColor(0, 128, 0));
    }

    QLabel *labelLed = new QLabel(strLedName);
    labelLed->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    labelLed->setFont(fontLabel);

    VBoxLayout->addWidget(btnLed);
    VBoxLayout->addWidget(labelLed);
    HBoxLayout->addLayout(VBoxLayout, 30);
    HBoxLayout->addWidget(btn, 70);

    ret.button = btn;
    ret.led = btnLed;
    ret.ledLabel = labelLed;
    ret.HBox = HBoxLayout;
    return ret;
}

void MainWindow::OnTimerInit()
{
    disconnect(timerInit, SIGNAL(timeout()), this, SLOT(OnTimerInit()));
}

void MainWindow::OnClickedButtonGetAdapter()
{

}

void MainWindow::OnClickedButtonGetSlaveInfo()
{

}

void MainWindow::OnClickedButtonRTnet()
{

}

void MainWindow::OnClickedButtonConnect()
{

}

void MainWindow::OnClickedButtonStart()
{

}

void MainWindow::OnClickedButtonStop()
{
    
}

void MainWindow::OnClickedButtonEcatStToInit()
{
    
}

void MainWindow::OnClickedButtonEcatStToPreOp()
{
    
}

void MainWindow::OnClickedButtonEcatStToSafeOp()
{
    
}

void MainWindow::OnClickedButtonEcatStToOp()
{
    
}

void MainWindow::OnClickedButtonLoopTest()
{

}

void MainWindow::OnTimer()
{
    
}

void MainWindow::OnClickButtonGetForceData()
{
    
}

void MainWindow::OnClickButtonGetTorqueData()
{

}

#define EC_TIMEOUTMON 500

void MainWindow::simpletest(char *ifname)
{
    
}

void MainWindow::close_ec_masters(void)
{

}

void MainWindow::signal_handler(int signo)
{
    
}

void MainWindow::print_usage(void)
{
   
}

