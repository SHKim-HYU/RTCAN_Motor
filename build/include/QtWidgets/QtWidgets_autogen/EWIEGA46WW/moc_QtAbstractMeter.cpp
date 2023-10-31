/****************************************************************************
** Meta object code from reading C++ file 'QtAbstractMeter.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "QtAbstractMeter.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtAbstractMeter.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QAbstractMeter_t {
    QByteArrayData data[16];
    char stringdata0[139];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QAbstractMeter_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QAbstractMeter_t qt_meta_stringdata_QAbstractMeter = {
    {
QT_MOC_LITERAL(0, 0, 14), // "QAbstractMeter"
QT_MOC_LITERAL(1, 15, 12), // "valueChanged"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 3), // "val"
QT_MOC_LITERAL(4, 33, 8), // "setValue"
QT_MOC_LITERAL(5, 42, 7), // "minimum"
QT_MOC_LITERAL(6, 50, 7), // "maximum"
QT_MOC_LITERAL(7, 58, 5), // "value"
QT_MOC_LITERAL(8, 64, 7), // "nominal"
QT_MOC_LITERAL(9, 72, 8), // "critical"
QT_MOC_LITERAL(10, 81, 6), // "prefix"
QT_MOC_LITERAL(11, 88, 6), // "suffix"
QT_MOC_LITERAL(12, 95, 9), // "valueFont"
QT_MOC_LITERAL(13, 105, 11), // "valueOffset"
QT_MOC_LITERAL(14, 117, 9), // "digitFont"
QT_MOC_LITERAL(15, 127, 11) // "digitOffset"

    },
    "QAbstractMeter\0valueChanged\0\0val\0"
    "setValue\0minimum\0maximum\0value\0nominal\0"
    "critical\0prefix\0suffix\0valueFont\0"
    "valueOffset\0digitFont\0digitOffset"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QAbstractMeter[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
      11,   46, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       1,    1,   37,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   40,    2, 0x0a /* Public */,
       4,    1,   43,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Double,    3,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Double,    3,

 // properties: name, type, flags
       5, QMetaType::Double, 0x00095103,
       6, QMetaType::Double, 0x00095103,
       7, QMetaType::Double, 0x00095103,
       8, QMetaType::Double, 0x00095103,
       9, QMetaType::Double, 0x00095103,
      10, QMetaType::QString, 0x00095103,
      11, QMetaType::QString, 0x00095103,
      12, QMetaType::QFont, 0x00095103,
      13, QMetaType::Double, 0x00095103,
      14, QMetaType::QFont, 0x00095103,
      15, QMetaType::Double, 0x00095103,

       0        // eod
};

void QAbstractMeter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QAbstractMeter *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->valueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->setValue((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->setValue((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QAbstractMeter::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractMeter::valueChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QAbstractMeter::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractMeter::valueChanged)) {
                *result = 1;
                return;
            }
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<QAbstractMeter *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< double*>(_v) = _t->minimum(); break;
        case 1: *reinterpret_cast< double*>(_v) = _t->maximum(); break;
        case 2: *reinterpret_cast< double*>(_v) = _t->value(); break;
        case 3: *reinterpret_cast< double*>(_v) = _t->nominal(); break;
        case 4: *reinterpret_cast< double*>(_v) = _t->critical(); break;
        case 5: *reinterpret_cast< QString*>(_v) = _t->prefix(); break;
        case 6: *reinterpret_cast< QString*>(_v) = _t->suffix(); break;
        case 7: *reinterpret_cast< QFont*>(_v) = _t->valueFont(); break;
        case 8: *reinterpret_cast< double*>(_v) = _t->valueOffset(); break;
        case 9: *reinterpret_cast< QFont*>(_v) = _t->digitFont(); break;
        case 10: *reinterpret_cast< double*>(_v) = _t->digitOffset(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<QAbstractMeter *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: _t->setMinimum(*reinterpret_cast< double*>(_v)); break;
        case 1: _t->setMaximum(*reinterpret_cast< double*>(_v)); break;
        case 2: _t->setValue(*reinterpret_cast< double*>(_v)); break;
        case 3: _t->setNominal(*reinterpret_cast< double*>(_v)); break;
        case 4: _t->setCritical(*reinterpret_cast< double*>(_v)); break;
        case 5: _t->setPrefix(*reinterpret_cast< QString*>(_v)); break;
        case 6: _t->setSuffix(*reinterpret_cast< QString*>(_v)); break;
        case 7: _t->setValueFont(*reinterpret_cast< QFont*>(_v)); break;
        case 8: _t->setValueOffset(*reinterpret_cast< double*>(_v)); break;
        case 9: _t->setDigitFont(*reinterpret_cast< QFont*>(_v)); break;
        case 10: _t->setDigitOffset(*reinterpret_cast< double*>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject QAbstractMeter::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidgetWithBackground::staticMetaObject>(),
    qt_meta_stringdata_QAbstractMeter.data,
    qt_meta_data_QAbstractMeter,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QAbstractMeter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QAbstractMeter::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QAbstractMeter.stringdata0))
        return static_cast<void*>(this);
    return QWidgetWithBackground::qt_metacast(_clname);
}

int QAbstractMeter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidgetWithBackground::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 11;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 11;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 11;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 11;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 11;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void QAbstractMeter::valueChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QAbstractMeter::valueChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
