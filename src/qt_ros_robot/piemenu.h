/*
COMPANY     : Cyient Limited.
MODULE      : QT UI Design
SUB MODULE  : Camera Page
DESCRIPTION : This is the header file for the PieMenu. It contains the definition for pie menu buttons implementation.
AUTHOR      : Malhar Sapatnekar
*/

#ifndef PIEMENU_H
#define PIEMENU_H

#include <QWidget>
#include <QPainterPath>
#include <QPainter>
#include <QDebug>
#include <QtMath>
#include <QMouseEvent>
#include <QLabel>
#include <QIcon>
#include <map>

/**
 * @brief The Segment enum defines the segments of the PieMenu.
 */
enum Segment {
    NoSegment,      /**< No specific segment */
    TopSegment,     /**< Top segment */
    BottomSegment,  /**< Bottom segment */
    LeftSegment,    /**< Left segment */
    RightSegment    /**< Right segment */
};

/**
 * @brief The PieMenu class is a custom QWidget that displays a pie-shaped menu.
 */
class PieMenu : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructs a PieMenu object.
     *
     * @param icon The icon to display in the center of the pie menu.
     * @param circleSize The size of the pie menu circle.
     * @param parent The parent widget (if any).
     */
    explicit PieMenu(QIcon icon, int circleSize, QWidget *parent = nullptr);

    /**
     * @brief Destroys the PieMenu object.
     */
    ~PieMenu();

    /**
     * @brief Sets the currently selected segment of the pie menu.
     *
     * @param segment The segment to set.
     */
    void setSegment(Segment segment);

    /**
     * @brief Update translation of the ui
     */
    void updateTranslations();

    /**
     * @brief Sets the currently selected segment and emit corresponding signal.
     *
     * @param segment The segment to set.
     */
    void handleButtonClick(Segment segment);


signals:
    /**
     * @brief Signal emitted when the top segment of the pie menu is clicked.
     */
    void topSegmentClicked();

    /**
     * @brief Signal emitted when the bottom segment of the pie menu is clicked.
     */
    void bottomSegmentClicked();

    /**
     * @brief Signal emitted when the left segment of the pie menu is clicked.
     */
    void leftSegmentClicked();

    /**
     * @brief Signal emitted when the right segment of the pie menu is clicked.
     */
    void rightSegmentClicked();

protected:
    /**
     * @brief Reimplemented from QWidget. Handles the paint event for drawing the pie menu.
     *
     * @param event The paint event.
     */
    void paintEvent(QPaintEvent *event);

    /**
     * @brief Reimplemented from QWidget. Handles mouse press events for segment selection.
     *
     * @param event The mouse event.
     */
    void mousePressEvent(QMouseEvent *event);

private:
    int m_circleSize; /**< Size of the pie menu circle. */
    Segment m_selectedSegment; /**< Currently selected segment of the pie menu. */
    QLabel* m_viewIconLabel; /**< Label to display the icon in the center of the pie menu. */

    QLabel* m_frontLabel;
    QLabel* m_leftLabel;
    QLabel* m_rightLabel;
    QLabel* m_rearLabel;
    bool m_translate;
    /**
     * @brief Map storing the angles defining each segment of the pie menu.
     */
    std::map<Segment, std::pair<int, int>> segmentAngles = {
        {TopSegment, {45, 135}},
        {LeftSegment, {135, 225}},
        {BottomSegment, {225, 315}},
        {RightSegment, {315, 405}}
    };

    /**
     * @brief Returns the segment corresponding to the given angle.
     *
     * @param angle The angle to determine the segment for.
     * @return The Segment enum value corresponding to the angle.
     */
    Segment getSegment(float angle);

    /**
     * @brief Checks if a mouse click occurred on the label.
     *
     * @param pos The position of the mouse click.
     * @param centerX The x-coordinate of the center of the label.
     * @param centerY The y-coordinate of the center of the label.
     * @return True if the click occurred on the label; false otherwise.
     */
    bool isClickOnLabel(QPoint pos, int centerX, int centerY);
};

#endif // PIEMENU_H
