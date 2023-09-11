
#include "pch.h"

#include <mfapi.h>
#include <mferror.h>
#include <string>
#include <chrono>

#define SPDLOG_FMT_EXTERNAL
#include <spdlog/spdlog.h>

#include <stdio.h>
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#include <windows.h>
#define sleep(x) Sleep(x * 1000)
#else
#include <unistd.h>
#endif
#include "zenoh.h"

#include <wmcodecdsp.h>

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Graphics.Imaging.h>

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices::Core;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;

#include "custom_buffers.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/msg/Hololens2VideoStream.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"

#define CHECK_HR(hr, msg) if (hr != S_OK) { printf(msg); printf(" Error: %.2X.\n", hr); goto done; }

template <class T> void SAFE_RELEASE(T * *ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

template <class T> inline void SAFE_RELEASE(T * &pT)
{
    if (pT != NULL)
    {
        pT->Release();
        pT = NULL;
    }
}


const char* kind_to_str(z_sample_kind_t kind);


/**
* Creates a new single buffer media sample.
* @param[in] bufferSize: size of the media buffer to set on the create media sample.
* @param[out] pSample: pointer to the create single buffer media sample.
* @@Returns S_OK if successful or an error code if not.
*/
HRESULT CreateSingleBufferIMFSample(DWORD bufferSize, IMFSample** pSample)
{
    IMFMediaBuffer* pBuffer = NULL;

    HRESULT hr = S_OK;

    hr = MFCreateSample(pSample);
    CHECK_HR(hr, "Failed to create MF sample.");

    // Adds a ref count to the pBuffer object.
    hr = MFCreateMemoryBuffer(bufferSize, &pBuffer);
    CHECK_HR(hr, "Failed to create memory buffer.");

    // Adds another ref count to the pBuffer object.
    hr = (*pSample)->AddBuffer(pBuffer);
    CHECK_HR(hr, "Failed to add sample to buffer.");

done:
    // Leave the single ref count that will be removed when the pSample is released.
    SAFE_RELEASE(pBuffer);
    return hr;
}


/**
* Attempts to get an output sample from an MFT transform.
* @param[in] pTransform: pointer to the media transform to apply.
* @param[out] pOutSample: pointer to the media sample output by the transform. Can be NULL
*  if the transform did not produce one.
* @param[out] transformFlushed: if set to true means the transform format changed and the
*  contents were flushed. Output format of sample most likely changed.
* @@Returns S_OK if successful or an error code if not.
*/
HRESULT GetTransformOutput(IMFTransform* pTransform, IMFSample** pOutSample, BOOL* transformFlushed)
{
    MFT_OUTPUT_STREAM_INFO StreamInfo = { 0 };
    MFT_OUTPUT_DATA_BUFFER outputDataBuffer = { 0 };
    DWORD processOutputStatus = 0;
    IMFMediaType* pChangedOutMediaType = NULL;

    HRESULT hr = S_OK;
    HRESULT mftProcessOutput = S_OK;
    *transformFlushed = FALSE;

    hr = pTransform->GetOutputStreamInfo(0, &StreamInfo);
    CHECK_HR(hr, "Failed to get output stream info from MFT.");

    outputDataBuffer.dwStreamID = 0;
    outputDataBuffer.dwStatus = 0;
    outputDataBuffer.pEvents = NULL;

    if ((StreamInfo.dwFlags & MFT_OUTPUT_STREAM_PROVIDES_SAMPLES) == 0) {
        hr = CreateSingleBufferIMFSample(StreamInfo.cbSize, pOutSample);
        CHECK_HR(hr, "Failed to create new single buffer IMF sample.");
        outputDataBuffer.pSample = *pOutSample;
    }

    mftProcessOutput = pTransform->ProcessOutput(0, 1, &outputDataBuffer, &processOutputStatus);

    //printf("Process output result %.2X, MFT status %.2X.\n", mftProcessOutput, processOutputStatus);

    if (mftProcessOutput == S_OK) {
        // Sample is ready and allocated on the transform output buffer.
        *pOutSample = outputDataBuffer.pSample;
    }
    else if (mftProcessOutput == MF_E_TRANSFORM_STREAM_CHANGE) {
        // Format of the input stream has changed. https://docs.microsoft.com/en-us/windows/win32/medfound/handling-stream-changes
        if (outputDataBuffer.dwStatus == MFT_OUTPUT_DATA_BUFFER_FORMAT_CHANGE) {
            printf("MFT stream changed.\n");

            hr = pTransform->GetOutputAvailableType(0, 0, &pChangedOutMediaType);
            CHECK_HR(hr, "Failed to get the MFT output media type after a stream change.");

            //std::cout << "MFT output media type: " << GetMediaTypeDescription(pChangedOutMediaType) << std::endl << std::endl;

            hr = pChangedOutMediaType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_IYUV);
            CHECK_HR(hr, "Failed to set media sub type.");

            hr = pTransform->SetOutputType(0, pChangedOutMediaType, 0);
            CHECK_HR(hr, "Failed to set new output media type on MFT.");

            hr = pTransform->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);
            CHECK_HR(hr, "Failed to process FLUSH command on MFT.");

            *transformFlushed = TRUE;
        }
        else {
            printf("MFT stream changed but didn't have the data format change flag set. Don't know what to do.\n");
            hr = E_NOTIMPL;
        }

        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
    }
    else if (mftProcessOutput == MF_E_TRANSFORM_NEED_MORE_INPUT) {
        // More input is not an error condition but it means the allocated output sample is empty.
        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
        hr = MF_E_TRANSFORM_NEED_MORE_INPUT;
    }
    else {
        printf("MFT ProcessOutput error result %.2X, MFT status %.2X.\n", mftProcessOutput, processOutputStatus);
        hr = mftProcessOutput;
        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
    }

done:

    SAFE_RELEASE(pChangedOutMediaType);

    return hr;
}



struct subscriber_context {
    const char* name{ nullptr };
    IMFTransform* pDecoderTransform = NULL; // This is H264 Decoder MFT.
    IMFMediaType* pDecInputMediaType = NULL; 
    IMFMediaType* pDecOutputMediaType = NULL;
    IUnknown* spDecTransformUnk = NULL;
    IMFMediaType* pMFTInputMediaType = NULL;
    IMFMediaType* pMFTOutputMediaType = NULL;
    DWORD mftStatus = 0;


    void init_context() {
    
        CHECK_HR(CoCreateInstance(CLSID_CMSH264DecoderMFT, NULL, CLSCTX_INPROC_SERVER,
            IID_IUnknown, (void**)&spDecTransformUnk), "Failed to create H264 decoder MFT.");

        CHECK_HR(spDecTransformUnk->QueryInterface(IID_PPV_ARGS(&pDecoderTransform)),
            "Failed to get IMFTransform interface from H264 decoder MFT object.");

        MFCreateMediaType(&pDecInputMediaType);
        CHECK_HR(pMFTOutputMediaType->CopyAllItems(pDecInputMediaType), "Error copying media type attributes to decoder input media type.");
        CHECK_HR(pDecoderTransform->SetInputType(0, pDecInputMediaType, 0), "Failed to set input media type on H.264 decoder MFT.");

        MFCreateMediaType(&pDecOutputMediaType);
        CHECK_HR(pMFTInputMediaType->CopyAllItems(pDecOutputMediaType), "Error copying media type attributes to decoder output media type.");
        CHECK_HR(pDecoderTransform->SetOutputType(0, pDecOutputMediaType, 0), "Failed to set output media type on H.264 decoder MFT.");

        CHECK_HR(pDecoderTransform->GetInputStatus(0, &mftStatus), "Failed to get input status from H.264 decoder MFT.");
        if (MFT_INPUT_STATUS_ACCEPT_DATA != mftStatus) {
            printf("H.264 decoder MFT is not accepting data.\n");
            goto done;
        }

        CHECK_HR(pDecoderTransform->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL), "Failed to process FLUSH command on H.264 decoder MFT.");
        CHECK_HR(pDecoderTransform->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, NULL), "Failed to process BEGIN_STREAMING command on H.264 decoder MFT.");
        CHECK_HR(pDecoderTransform->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, NULL), "Failed to process START_OF_STREAM command on H.264 decoder MFT.");

    done:

        // noop
        printf("init completed");
    }

    void data_handler(const z_sample_t* sample) {
        using namespace std::chrono;

        z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
        spdlog::info("Subscriber Received {0}, len: {1}", kind_to_str(sample->kind), (int)sample->payload.len);

        char* buffer_start = const_cast<char*>(reinterpret_cast<const char*>(sample->payload.start)); // not sure why we need to const_cast here .. we won't modify the buffer ..
        eprosima::fastcdr::FastBuffer cdrbuffer(buffer_start, sample->payload.len);
        eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
        cdr_des.read_encapsulation();

        pcpd_msgs::msg::Hololens2VideoStream msg{};
        cdr_des >> msg;

        spdlog::info("Got Measurement with TS: {0}[{1}] and size: {2}", msg.header().stamp().sec(), msg.header().stamp().nanosec(), msg.image_bytes());

        IMFSample* pSample; // Release
        WrappedBuffer* pBuffer; // Release

        IMFSample* pH264DecodeOutSample = NULL;
        BOOL h264DecodeTransformFlushed = FALSE;

        HRESULT getDecoderResult = S_OK;


        // create IMF Buffer from received payload
        WrappedBuffer::CreateInstance(&pBuffer, msg.image().data(), static_cast<DWORD>(msg.image().size()));

        // create Sample using IMF Buffer
        MFCreateSample(&pSample);

        uint64_t timestamp = duration_cast<nanoseconds>(seconds(msg.header().stamp().sec()) + nanoseconds(msg.header().stamp().nanosec())).count();

        pSample->AddBuffer(pBuffer);
        pSample->SetSampleDuration(static_cast<LONGLONG>(1.0/15.0)); // where do we get that from ??
        pSample->SetSampleTime(timestamp);

        // Apply the H264 decoder transform
        CHECK_HR(pDecoderTransform->ProcessInput(0, pSample, 0),
            "The H264 decoder ProcessInput call failed.");

        // ----- H264 DEcoder transform processing loop. -----

        while (getDecoderResult == S_OK) {

            // Apply the H264 decoder transform
            getDecoderResult = GetTransformOutput(pDecoderTransform, &pH264DecodeOutSample, &h264DecodeTransformFlushed);

            if (getDecoderResult != S_OK && getDecoderResult != MF_E_TRANSFORM_NEED_MORE_INPUT) {
                printf("Error getting H264 decoder transform output, error code %.2X.\n", getDecoderResult);
                goto done;
            }

            if (h264DecodeTransformFlushed == TRUE) {
                // H264 decoder format changed. Clear the capture file and start again.
                printf("H264 decoder transform flushed stream.\n");

                // recreate texture here?

            }
            else if (pH264DecodeOutSample != NULL) {
                // Write decoded sample to capture file.
                //CHECK_HR(WriteSampleToFile(pH264DecodeOutSample, &outputBuffer),
                //    "Failed to write sample to file.");

                // so someting with pH264DecodeOutSample
            }

            SAFE_RELEASE(pH264DecodeOutSample);
        }
        // -----


        done:
        z_drop(z_move(keystr));
    }


    void teardown_context() {
        SAFE_RELEASE(pMFTInputMediaType);
        SAFE_RELEASE(pMFTOutputMediaType);
        SAFE_RELEASE(spDecTransformUnk);
        SAFE_RELEASE(pDecoderTransform);
        SAFE_RELEASE(pDecInputMediaType);
        SAFE_RELEASE(pDecOutputMediaType);
    }

};


void handle_subscriber_callback(const z_sample_t* sample, void* context) {
    auto handle = static_cast<subscriber_context*>(context);
    if (handle != nullptr && sample != nullptr) {
        try {
            handle->data_handler(sample);
        }
        catch (const std::exception& e) {
            SPDLOG_ERROR("Error during subscription callback for: {0} -> {1}", handle->name, e.what());
        }
    }
    else {
        SPDLOG_ERROR("Invalid handle or sample during subscription callback.");
    }
}

void free_subscriber_context(void* context) {
    if (context != nullptr) {
        free(context);
    }
}

int main(int argc, char** argv) {
    char* expr;
    if (argc > 1) {
        expr = argv[1];
    }
    else {
        spdlog::error("Missing topic name as argument");
        exit(1);
    }

    z_owned_config_t config = z_config_default();
    if (argc > 2) {
        if (zc_config_insert_json(z_loan(config), Z_CONFIG_LISTEN_KEY, argv[2]) < 0) {
            printf(
                "Couldn't insert value `%s` in configuration at `%s`. This is likely because `%s` expects a "
                "JSON-serialized list of strings\n",
                argv[2], Z_CONFIG_LISTEN_KEY, Z_CONFIG_LISTEN_KEY);
            exit(-1);
        }
    }

    printf("Opening session...\n");
    z_owned_session_t s = z_open(z_move(config));
    if (!z_check(s)) {
        printf("Unable to open session!\n");
        exit(-1);
    }

    auto ctx = new subscriber_context();
    ctx->name = "video_subscriber";

    // zclosure macro does not work with c++17
    z_owned_closure_sample_t callback{};
    callback.call = &handle_subscriber_callback;
    callback.context = ctx;
    callback.drop = free_subscriber_context;

    auto options = z_subscriber_options_default();

    ctx->init_context();
    
    printf("Declaring Subscriber on '%s'...\n", expr);
    z_owned_subscriber_t sub = z_declare_subscriber(z_loan(s), z_keyexpr(expr), z_move(callback), &options);
    if (!z_check(sub)) {
        printf("Unable to declare subscriber.\n");
        exit(-1);
    }

    printf("Enter 'q' to quit...\n");
    char c = 0;
    while (c != 'q') {
        c = getchar();
        if (c == -1) {
            sleep(1);
        }
    }

    z_undeclare_subscriber(z_move(sub));
    z_close(z_move(s));

    ctx->teardown_context();

    return 0;
}

const char* kind_to_str(z_sample_kind_t kind) {
    switch (kind) {
    case Z_SAMPLE_KIND_PUT:
        return "PUT";
    case Z_SAMPLE_KIND_DELETE:
        return "DELETE";
    default:
        return "UNKNOWN";
    }
}