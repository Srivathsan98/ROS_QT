#ifndef DIRECTION_MENU_H
#define DIRECTION_MENU_H
#include <QWidget>
#include <QVector>
#include <QString>

class DirectionMenu : public QWidget
{
    Q_OBJECT

public:
    explicit DirectionMenu(QWidget *parent = nullptr);
    void setItems(const QVector<QString> &items);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

signals:
    void clicked(int value);

private:
    QVector<QString> menuItems;
    int hoveredIndex = -1;

    int itemAt(const QPoint &pos) const;
};

#endif // DIRECTION_MENU_H
