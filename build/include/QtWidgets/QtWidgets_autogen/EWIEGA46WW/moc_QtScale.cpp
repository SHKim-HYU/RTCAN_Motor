/****************************************************************************
** Meta object code from reading C++ file 'QtScale.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "QtScale.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtScale.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QScale_t {
    QByteArrayData data[12];
    char stringdata0[124];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QScale_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QScale_t qt_meta_stringdata_QScale = {
    {
QT_MOC_LITERAL(0, 0, 6), // "QScale"
QT_MOC_LITERAL(1, 7, 10), // "setMinimum"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 10), // "setMaximum"
QT_MOC_LITERAL(4, 30, 8), // "setRange"
QT_MOC_LITERAL(5, 39, 3), // "min"
QT_MOC_LITERAL(6, 43, 3), // "max"
QT_MOC_LITERAL(7, 47, 8), // "setValue"
QT_MOC_LITERAL(8, 56, 16), // "setMajorStepSize"
QT_MOC_LITERAL(9, 73, 17), // "setMinorStepCount"
QT_MOC_LITERAL(10, 91, 15), // "setOrientations"
QT_MOC_LITERAL(11, 107, 16) // "Qt::Orientations"

    },
    "QScale\0setMinimum\0\0setMaximum\0setRange\0"
    "min\0max\0setValue\0setMajorStepSize\0"
    "setMinorStepCount\0setOrientations\0"
    "Qt::Orientations"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QScale[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x0a /* Public */,
       3,    1,   52,    2, 0x0a /* Public */,
       4,    2,   55,    2, 0x0a /* Public */,
       7,    1,   60,    2, 0x0a /* Public */,
       8,    1,   63,    2, 0x0a /* Public */,
       9,    1,   66,    2, 0x0a /* Public */,
      10,    1,   69,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    5,    6,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, 0x80000000 | 11,    2,

       0        // eod
};

void QScale::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QScale *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setMinimum((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 1: _t->setMaximum((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->setRange((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 3: _t->setValue((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->setMajorStepSize((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->setMinorStepCount((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->setOrientations((*reinterpret_cast< Qt::Orientations(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QScale::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_QScale.data,
    qt_meta_data_QScale,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QScale::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QScale::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QScale.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int QScale::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
