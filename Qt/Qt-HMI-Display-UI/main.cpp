#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "radialbar.h"
#include "canreceiver.h"  // thêm dòng này
#include <QQmlContext>

int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    qmlRegisterType<RadialBar>("CustomControls", 1, 0, "RadialBar");

    CanReceiver canReceiver;
    engine.rootContext()->setContextProperty("canReceiver", &canReceiver);  // expose to QML

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
        &app, [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        }, Qt::QueuedConnection);
    engine.load(url);

    return app.exec();
}
