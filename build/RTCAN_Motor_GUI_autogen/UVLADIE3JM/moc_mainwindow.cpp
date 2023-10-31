/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "src/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[19];
    char stringdata0[383];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 20), // "OnClickedButtonRTnet"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 22), // "OnClickedButtonConnect"
QT_MOC_LITERAL(4, 56, 20), // "OnClickedButtonStart"
QT_MOC_LITERAL(5, 77, 19), // "OnClickedButtonStop"
QT_MOC_LITERAL(6, 97, 27), // "OnClickedButtonEcatStToInit"
QT_MOC_LITERAL(7, 125, 28), // "OnClickedButtonEcatStToPreOp"
QT_MOC_LITERAL(8, 154, 29), // "OnClickedButtonEcatStToSafeOp"
QT_MOC_LITERAL(9, 184, 25), // "OnClickedButtonEcatStToOp"
QT_MOC_LITERAL(10, 210, 25), // "OnClickedButtonGetAdapter"
QT_MOC_LITERAL(11, 236, 27), // "OnClickedButtonGetSlaveInfo"
QT_MOC_LITERAL(12, 264, 23), // "OnClickedButtonLoopTest"
QT_MOC_LITERAL(13, 288, 11), // "OnTimerInit"
QT_MOC_LITERAL(14, 300, 7), // "OnTimer"
QT_MOC_LITERAL(15, 308, 25), // "OnClickButtonGetForceData"
QT_MOC_LITERAL(16, 334, 26), // "OnClickButtonGetTorqueData"
QT_MOC_LITERAL(17, 361, 12), // "OnLogMessage"
QT_MOC_LITERAL(18, 374, 8) // "QString&"

    },
    "MainWindow\0OnClickedButtonRTnet\0\0"
    "OnClickedButtonConnect\0OnClickedButtonStart\0"
    "OnClickedButtonStop\0OnClickedButtonEcatStToInit\0"
    "OnClickedButtonEcatStToPreOp\0"
    "OnClickedButtonEcatStToSafeOp\0"
    "OnClickedButtonEcatStToOp\0"
    "OnClickedButtonGetAdapter\0"
    "OnClickedButtonGetSlaveInfo\0"
    "OnClickedButtonLoopTest\0OnTimerInit\0"
    "OnTimer\0OnClickButtonGetForceData\0"
    "OnClickButtonGetTorqueData\0OnLogMessage\0"
    "QString&"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x0a /* Public */,
       3,    0,   95,    2, 0x0a /* Public */,
       4,    0,   96,    2, 0x0a /* Public */,
       5,    0,   97,    2, 0x0a /* Public */,
       6,    0,   98,    2, 0x0a /* Public */,
       7,    0,   99,    2, 0x0a /* Public */,
       8,    0,  100,    2, 0x0a /* Public */,
       9,    0,  101,    2, 0x0a /* Public */,
      10,    0,  102,    2, 0x0a /* Public */,
      11,    0,  103,    2, 0x0a /* Public */,
      12,    0,  104,    2, 0x0a /* Public */,
      13,    0,  105,    2, 0x0a /* Public */,
      14,    0,  106,    2, 0x0a /* Public */,
      15,    0,  107,    2, 0x0a /* Public */,
      16,    0,  108,    2, 0x0a /* Public */,
      17,    1,  109,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 18,    2,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->OnClickedButtonRTnet(); break;
        case 1: _t->OnClickedButtonConnect(); break;
        case 2: _t->OnClickedButtonStart(); break;
        case 3: _t->OnClickedButtonStop(); break;
        case 4: _t->OnClickedButtonEcatStToInit(); break;
        case 5: _t->OnClickedButtonEcatStToPreOp(); break;
        case 6: _t->OnClickedButtonEcatStToSafeOp(); break;
        case 7: _t->OnClickedButtonEcatStToOp(); break;
        case 8: _t->OnClickedButtonGetAdapter(); break;
        case 9: _t->OnClickedButtonGetSlaveInfo(); break;
        case 10: _t->OnClickedButtonLoopTest(); break;
        case 11: _t->OnTimerInit(); break;
        case 12: _t->OnTimer(); break;
        case 13: _t->OnClickButtonGetForceData(); break;
        case 14: _t->OnClickButtonGetTorqueData(); break;
        case 15: _t->OnLogMessage((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
