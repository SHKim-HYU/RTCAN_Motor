
#if (1)
#include "mainwindow.h"
#include <iostream>

#include <QApplication>
#include <QResource>
#include <QTextCodec>
//#include <QDesktopWidget>
#include <QScreen>

#include "DarkStyle.h"
#include "framelesswindow.h"


using namespace std;

#define DEF_DARKSTYLE  2
int main(int argc, char* argv[])
{
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));

    QApplication a(argc, argv);

    QFont font = a.font();
    font.setStyleHint(QFont::Fantasy);
    // font.setFamily("Verdana");
    font.setFamily("Arial");

    QScreen* screen = QGuiApplication::primaryScreen();
    QRect screenres = screen->availableGeometry();
   
    //font.setWeight(QFont::Bold);
    a.setFont(font);
    
    //a.setFont(QFont("Verdana", 8, QFont::Bold));
    //a.setStyle(QStyleFactory::create("fusion"));
    //a.setStyle(new QMacStyle);
#if(DEF_DARKSTYLE==1)
    // style our application with custom dark style
    QApplication::setStyle(new DarkStyle);

    // create frameless window (and set windowState or title)
    FramelessWindow framelessWindow;
    //framelessWindow.setWindowState(Qt::WindowFullScreen);
    framelessWindow.setWindowTitle("Robot Control Software | Simple Open EtherCAT Master (SOEM)");
    framelessWindow.setWindowIcon(QIcon("app.ico"));

    // create our mainwindow instance
    MainWindow* w = new MainWindow;
    framelessWindow.setFont(font);

    // add the mainwindow to our custom frameless window
    framelessWindow.setContent(w);

    // screenGeometry 변수의 파라미터 변경시, 다른 모니터의 정보를 가져올 수 있다.
    //QRect screenres = QApplication::desktop()->screenGeometry(1/*screenNumber*/);
    // Second monitor의 좌표로 이동
    //framelessWindow.move(screenres.width() / 4, screenres.height() / 4);
    framelessWindow.move(0, 0);
    // Second monitor의 크기만큼 Window의 크기 조정
    framelessWindow.resize(screenres.width()/2, screenres.height()/2);
    // full 스크린으로 사용할 때 사용
    //framelessWindow.showFullScreen();

    //framelessWindow.resize(1920, 1000);
    //framelessWindow.move(0, 0);
    framelessWindow.show();
#elif(DEF_DARKSTYLE==2)    
    QResource::registerResource("darkstyle.qrc");
    QFile file(":/darkstyle/darkstyle.qss");
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        a.setStyleSheet(file.readAll());
        file.close();
    }

    // style our application with custom dark style
    QApplication::setStyle(new DarkStyle);
    QApplication::setPalette(QApplication::style()->standardPalette());

    MainWindow w;
    w.setWindowTitle("Robot Control Software | CANopen CiA402");
    w.setWindowIcon(QIcon("app.ico"));

    // Second monitor의 좌표로 이동
    //framelessWindow.move(screenres.width() / 4, screenres.height() / 4);
    w.move(0, 0);
    // Second monitor의 크기만큼 Window의 크기 조정
    w.resize(screenres.width()/1, screenres.height()/1.5);
    // full 스크린으로 사용할 때 사용
    //framelessWindow.showFullScreen();

    //framelessWindow.resize(1920, 1000);
    //framelessWindow.resize(1280/2, 1024/2);
    //framelessWindow.move(0, 0);

    w.show();
#else
    // create our mainwindow instance
    Laserssel w;
    w.setWindowTitle("Laser Diode Control and Monitoring S/W");
    w.show();
#endif

    return a.exec();
}
#else
#include <QApplication>

#include <QSplitter>

#include <QTreeView>
#include <QListView>
#include <QTableView>

#include <QStandardItemModel>

int main( int argc, char **argv )
{
  QApplication app( argc, argv );
  
  QTreeView *tree = new QTreeView();
  QListView *list = new QListView();
  QTableView *table = new QTableView();
  
  QSplitter splitter;
  splitter.addWidget( tree );
  splitter.addWidget( list );
  splitter.addWidget( table );
  
  QStandardItemModel model( 5, 2 );
  for( int r=0; r<5; r++ ) 
    for( int c=0; c<2; c++) 
    {
      QStandardItem *item = new QStandardItem( QString("Row:%0, Column:%1").arg(r).arg(c) );
      
      if( c == 0 )
        for( int i=0; i<3; i++ )
        {
          QStandardItem *child = new QStandardItem( QString("Item %0").arg(i) );
          child->setEditable( false );
          item->appendRow( child );
        }
      
      model.setItem(r, c, item);
    }

  model.setHorizontalHeaderItem( 0, new QStandardItem( "Foo" ) );
  model.setHorizontalHeaderItem( 1, new QStandardItem( "Bar-Baz" ) );

  tree->setModel( &model );
  list->setModel( &model );
  table->setModel( &model );

  list->setSelectionModel( tree->selectionModel() );
  table->setSelectionModel( tree->selectionModel() );

  table->setSelectionBehavior( QAbstractItemView::SelectRows );
  table->setSelectionMode( QAbstractItemView::SingleSelection );

  splitter.show();
  
  return app.exec();
}
#endif