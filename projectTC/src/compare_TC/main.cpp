#include <QCoreApplication>
#include <QtConcurrent>
#include <QElapsedTimer>
#include <functional>
#include <QThread>
#include <QMutex>
#include <QFutureWatcher>
#include <QFuture>
#include <iostream>

#include "CtrlAir_Ballist_structs.h"
#include "CtrlAir_Ballist.h"
#include "CtrlAir_Ballist_M.h"

#include "load_input_data.h"

QMutex mutex;

/*class AirBallistWorker : public QThread
{
    public:
        AirBallistWorker(CtrlAIR_BALLIST& processor, const CtrlAIR_BALL_Input_ST_Data& data) : m_processor (processor), m_data(data)
        {

        }

    protected:
            void run() override
            {
                m_processor.ProcessingAirBallist(m_data);
            }

   private:
            CtrlAIR_BALLIST& m_processor;
            CtrlAIR_BALL_Input_ST_Data m_data;

};*/



namespace input_data_dir {
    const char* pWorkingFolder = "../data/input";
    const char* pViewFolder = "../data/view";
}

template<class proc_class>
void proc_data(proc_class& processor, quint8 threads, const load_input_data::arr_input_data& input_data);

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    // create data for test
    std::string l_data_file("Input_AIR_BALL.log"), l_reference_file("RD_ID_AIR_BALL.log");
    load_input_data l_load_input_data(l_data_file.data(), l_reference_file.data());

    AIR_BALL::s_arCovObj l_arrCovObj;
    Str_Ar_EAP_Param_AllTables l_arrEAP_Tables;
    l_load_input_data.ParseRDIDFile(l_arrCovObj, l_arrEAP_Tables);
    std::cout << "Reference data is loaded" << std::endl << std::flush;

    load_input_data::arr_input_data l_arrInputData;
    std::cout << "Loading input data ... " << std::flush;
    l_load_input_data.ParseInputFile(l_arrInputData);
    std::cout << "completed" << std::endl << std::flush;

    // create processing class
    CtrlAIR_BALLIST l_one_thread_TC;
    l_one_thread_TC.FillCovObj(l_arrCovObj);
    l_one_thread_TC.SetPersistentData(l_arrEAP_Tables);

    // processing input data

    //proc_data(l_one_thread_TC, 1, l_arrInputData);
    proc_data(l_one_thread_TC, 2, l_arrInputData);
    CtrlAIR_BALLIST_M l_multy_thread_TC;
    l_multy_thread_TC.FillCovObj(l_arrCovObj);
    l_multy_thread_TC.SetPersistentData(l_arrEAP_Tables);

    //proc_data(l_multy_thread_TC, 1, l_arrInputData);
    proc_data(l_multy_thread_TC, 2, l_arrInputData);

    //return a.exec();
    return 0;
}

/*template<class proc_class>
void proccAirBallistThreadSafe(proc_class& processor, const CtrlAIR_BALL_Input_ST_Data& InData)
{
    QMutexLocker locker(&mutex);
    processor.ProcessingAirBallist(InData);
}
*/

/*template<class proc_class>
void runParallelProc(proc_class& processor, const std::vector<DataType>& datalist)
{
    QVector<QFuture<void>> futures;
    QFutureWatcher<void> watcher;

    for(const auto& data:datalist)
    {
        QFuture<void> future = QtConcurrent::run(proccAirBallistThreadSafe, std::ref(processor));
        futures.append(future);
    }

    for (auto& future : futures)
    {
        future.waitForFinished();
    }
}
*/

template<class proc_class>
void proc_data(proc_class& processor, quint8 threads, const load_input_data::arr_input_data& input_data)
{
    bool l_one_thread(true);
    if( !std::is_same<decltype(processor), CtrlAIR_BALLIST&>::value ) {
        l_one_thread = false;
    }

    QString l_proc_type(l_one_thread ? "one_thread" : "multy_thread");
    l_proc_type += threads > 1 ? "_parallel" :"_successive";

    QString l_fileName = QString("%1/compare_%2.log").arg(input_data_dir::pViewFolder).arg(l_proc_type);
    QTextStream l_streamCompare(stdout);

    QFile l_fileCompare(l_fileName);
    if( l_fileCompare.open(QIODevice::WriteOnly) ) {
        l_streamCompare.setDevice(&l_fileCompare);
    }
    auto l_pLogPolynom = GLFileLog::OpenLog("PolynomData", input_data_dir::pViewFolder);

    // timer for check processing duration
    QElapsedTimer l_timer;

    // process track data
    std::cout << "Processing input data by " << qPrintable(l_proc_type) << " " << QThread::idealThreadCount() << std::flush;
    std::cout << " : processor object size " << qreal(sizeof(processor)) /1024. << "kb" << "\t" << std::flush; 
    qint64 l_proc_rate(200), l_time_rate(100);  //msec
    auto l_proc_time = input_data.begin()->first;
    auto l_stop_time = input_data.rbegin()->first;
    std::vector<CtrlAIR_BALL_Input_GT_Data> l_arr_inputGT;
    l_arr_inputGT.reserve(AIR_BALL::GT_FORM_AMOUNT);

    // timer for all processing cycle
    quint64 l_all_time(0);
   // std::mutex dataMutex;
    auto l_data = input_data.begin();
    auto l_curr_time = l_proc_time;
    while( l_proc_time <= l_stop_time)
    {
        l_proc_time += l_proc_rate;
        std::cout << "." << std::flush;

        QString l_log = QString("Proc time = %1 (%2) Proc duration for %3 processing ST")
                .arg(l_proc_time).arg(MSEC70_to_SEC_START_OF_DAY(l_proc_time), 0 , 'f', 2).arg(l_proc_type);

        l_arr_inputGT.clear();

        QFutureSynchronizer<void> l_future_ST;
        quint32 l_count_ST(0);

        // start processing ST data and storing GT data
        qint64 l_all_wait_time(0);
        l_timer.start();

        while( l_data->first < l_proc_time && l_data != input_data.end() )
        {
            double l_time = (double)l_data->first / 1000.;
            processor.SetCurTime(l_time);

            qint64 l_wait_time = (l_data->first - l_curr_time) / l_time_rate;
            if( l_wait_time > 0 ) {
                l_curr_time = l_data->first;
                l_all_wait_time += l_wait_time;
                QThread::msleep(l_wait_time);
            }

            switch( l_data->second.type )
            {
            // ST to be processed immediately from different threads
            case load_input_data::input_data::type_ST:
            {

                if( threads > 1 )
                {
                    std::function<bool(const CtrlAIR_BALL_Input_ST_Data &)> l_funcProcST =
                    [&processor] (const CtrlAIR_BALL_Input_ST_Data& InData) -> bool
                    //[&processor, &dataMutex] (const CtrlAIR_BALL_Input_ST_Data& InData) -> bool
                    {
                        //std::lock_guard<std::mutex> lock(dataMutex);
                        processor.ProcessingAirBallist(InData);
                        return true;
                    };
                    l_future_ST.addFuture(QtConcurrent::run(l_funcProcST, l_data->second.data_ST));
                }
                else {
                    processor.ProcessingAirBallist(l_data->second.data_ST);
                }
                ++l_count_ST;
                break;
            }
            // GT to be stored for l_time_rate, then processed
            case load_input_data::input_data::type_GT:
            {
                l_arr_inputGT.push_back(l_data->second.data_GT);
                break;
            }
            // MSG to be added immediately
            case load_input_data::input_data::type_MSG:
            {
                processor.m_InpMsgsBuf.AddMsg(l_data->second.data_MSG);
                break;
            }
            default: break;
            }
            ++l_data;
        }
        l_future_ST.waitForFinished();

        // fix duration of ST data processing
        auto l_time = l_timer.nsecsElapsed();
        l_time -= l_all_wait_time *1.e+6;
        l_all_time += l_time;
        l_log += QString("(%1) %2msec | processing GT(%3) ").arg(l_count_ST)
                .arg((qreal)l_time/ 1.e+6, 0, 'f', 5).arg(l_arr_inputGT.size());

        // function for GT processing
        std::function<bool(const CtrlAIR_BALL_Input_GT_Data&)> l_funcProcGT =
                [&processor, &l_pLogPolynom](const CtrlAIR_BALL_Input_GT_Data& InData) -> bool
        {
            CtrlAIR_BALL_Output_GT_Data l_outGT;
            quint32 l_repeat(0);
            while( !processor.ProcessingAirBallist(InData, l_outGT) ) {
                ++l_repeat;
            }

            if (l_outGT.bNewPrediction || l_outGT.bNewPolynomSat)
            {
                char NumString[1000];
                sprintf(NumString, "%d %f %f %f %f %f "
                                   "%f %f %f %f %f %f %f",
                        InData.NumbGT, InData.tLoc, InData.X, InData.Y, InData.Z,
                        l_outGT.Predict.tStart, l_outGT.Predict.StartPoint.x, l_outGT.Predict.StartPoint.y, l_outGT.Predict.StartPoint.z,
                        l_outGT.Predict.tFall, l_outGT.Predict.FallPoint.x, l_outGT.Predict.FallPoint.y, l_outGT.Predict.FallPoint.z);
                l_outGT.Predict.Pol.LogData(l_pLogPolynom, NumString); //output polynom
            }
            return true;
        };

        // start GT data processing
        QFutureSynchronizer<void> l_future_GT;
        l_timer.start();

        if( threads > 1 ) {
            for(auto &l_inputGT : l_arr_inputGT ) {
                l_future_GT.addFuture(QtConcurrent::run(l_funcProcGT, l_inputGT));
            }

        //   auto l_future_GT = QtConcurrent::mapped(l_arr_inputGT, l_funcProcGT);
            l_future_GT.waitForFinished();
        }
        else {
            for(auto &l_inputGT : l_arr_inputGT ) {
                l_funcProcGT(l_inputGT);
            }
        }
        l_time = l_timer.nsecsElapsed();
        l_all_time += l_time;

        l_log += QString("%1msec \n").arg((qreal)l_time/ 1.e+6, 0, 'f', 5);
        (l_streamCompare << l_log).flush();
    }
    std::cout << " completed in " << (qreal)l_all_time / 1.e+6 << "msec" << std::endl << std::flush;

    if( l_fileCompare.isOpen() ) {
        l_fileCompare.close();
    }
    if( l_pLogPolynom ) {
        fclose(l_pLogPolynom);
    }
}

