#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
// #include <QDebug>
#include <QTimer>

#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>
#include <QCloseEvent>
#include <QMessageBox>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QTreeWidget>

#include "QtButtonLedIndicator.h"

#include <vector>

// #include "EcatMaster.h"
//#include <type_traits>
#include <chrono>

typedef struct {
    QHBoxLayout*            HBox;
    QLabel*                 ledLabel;
    QButtonLedIndicator*    led;
}Led_t;

typedef struct {
    QHBoxLayout*            HBox;
    QLabel*                 ledLabel;
    QPushButton*            button;
    QButtonLedIndicator*    led;
}LedButton_t;

#define DEF_MAX_SERVO   7
typedef struct {
    QHBoxLayout*            HBox;
    QLabel*                 modeLabel;
    QLabel*                 labelTargetPos;
    QLineEdit*              editTargetPos;
    QLabel*                 labelActualPos;
    QLineEdit*              editActualPos;
    QLabel*                 labelTargetVel;
    QLineEdit*              editTargetVel;
    QLabel*                 labelActualVel;
    QLineEdit*              editActualVel;
    QLabel*                 labelTargetTor;
    QLineEdit*              editTargetTor;
    QLabel*                 labelActualTor;
    QLineEdit*              editActualTor;
    QPushButton*            btnMove;
    QPushButton*            btnJogNDir;
    QPushButton*            btnJogPDir;    
}Servo_t;

typedef struct {
    //QHBoxLayout*            HBox;
    LedButton_t             ledButtonServo;
    QPushButton*            Halt;
    QPushButton*            QStop;    
    QPushButton*            Move;
    QPushButton*            ResetFault;
    QPushButton*            GoHome;    
}ServoControl_t;



class Stopwatch final
{
public:

    using elapsed_resolution = std::chrono::nanoseconds;

    Stopwatch()
    {
        Reset();
    }

    void Reset()
    {
        reset_time = clock.now();
    }

    elapsed_resolution Elapsed()
    {
        return std::chrono::duration_cast<elapsed_resolution>(clock.now() - reset_time);
    }

private:

    std::chrono::high_resolution_clock clock;
    std::chrono::high_resolution_clock::time_point reset_time;
};




using namespace std;  

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

typedef struct {
    QHBoxLayout* HBox;
    QLabel* ledLabel;
    QPushButton* button;
    QButtonLedIndicator* led;
}MainV1Group;

typedef struct {
    Led_t        ledEcatState;
    QPushButton* btnStInit;
    QPushButton* btnStPreOp;
    QPushButton* btnStSafeOP;
    QPushButton* btnStOp;
}EcatStBtn_t;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event);
    
private:
    Ui::MainWindow *ui;

    vector<QString> logMsgBuff;
    QTextEdit*      textEditLogMsg;
    QTreeWidget*    tree_SlaveList;

    QLineEdit*      lineEditCycleTime;
    QLineEdit*      lineEditexeTime;

    QTimer          *timer;    
    QTimer          *timerInit;

    QWidget         *chart;
    void resizeEvent(QResizeEvent *);

public:
    // CEcatMaster ecat;
    
    void LogMessage(const char* sz, ...);
    void LogMessage(QString& sz);
    void LogMessagePrint();
    void AddTreeRootNode(const QString& value, const QString& desc);

    Servo_t jtServo[DEF_MAX_SERVO];
    Servo_t CreateJointServoLayout(bool header=false);

    ServoControl_t  jtServoControl;

    EcatStBtn_t     ecatStateBtn;
    void EcatStateButton(uint16_t state);
    typedef enum { LED_RED, LED_GREEN, LED_YELLOW }led_color;
    LedButton_t LedButtonLayout(QString strBtnName, QString strLedName, led_color co, bool btnVisible);
    Led_t LedLayout(QString strLedName, QString strLedLabel, led_color co);

    char ifbuf[1024];

    unsigned long cycleTime;
    unsigned long exeTime;

    // OSAL_THREAD_HANDLE thread_ecatcheck;
    // OSAL_THREAD_HANDLE thread_ecatprocess;

    char hstr[1024];

    void close_ec_masters(void);
    // most basic RT thread for process data, just does IO transfer
    //void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);
    //void ecatthread(void* arg);
    // thread function to check ethercat status
    //OSAL_THREAD_FUNC MainWindow::ecatcheck(void* arg);
    void signal_handler(int signo);
    void print_usage(void);

    uint8_t  currentgroup;
    bool needlf;
    volatile int wkc;


    void simpletest(char *ifname);
    //static OSAL_THREAD_FUNC ecatcheck( void *ptr );


    unsigned long _loop_cnt;
    unsigned long _loop_fail_cnt;
    uint8_t _u8out_a;
    uint8_t _u8in_a;
    uint8_t _u8out_b;
    uint8_t _u8in_b;

public slots:
    void OnClickedButtonRTnet();
    void OnClickedButtonConnect();
    void OnClickedButtonStart();
    void OnClickedButtonStop();    

    void OnClickedButtonEcatStToInit();
    void OnClickedButtonEcatStToPreOp();
    void OnClickedButtonEcatStToSafeOp();
    void OnClickedButtonEcatStToOp();


    void OnClickedButtonGetAdapter();
    void OnClickedButtonGetSlaveInfo();
    void OnClickedButtonLoopTest();

    void OnTimerInit();
    void OnTimer();

    void OnClickButtonGetForceData();
    void OnClickButtonGetTorqueData();

    void OnLogMessage(QString &);
};
#endif // MAINWINDOW_H
