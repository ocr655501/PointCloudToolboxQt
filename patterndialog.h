#ifndef PATTERNDIALOG_H
#define PATTERNDIALOG_H

#include <QDialog>

namespace Ui {
class PatternDialog;
}

class PatternDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PatternDialog(QWidget *parent = nullptr);
    ~PatternDialog();

private:
    Ui::PatternDialog *ui;

private slots:
    void on_ApplyR_clicked();
    void on_Apply01_clicked();
    void on_AboutQt_clicked();
    void on_ApplyM_clicked();
    void on_APplyC_clicked();
    void on_ApplyMisc_clicked();
};

#endif // PATTERNDIALOG_H
