#include "pch.h"
#include "HoloCameraStreamer.h"

using namespace winrt;
using namespace winrt::Windows::Foundation;

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;

static constexpr winrt::guid c_MF_MT_USER_DATA(
	0xb6bc765f, 0x4c3b, 0x40a4, { 0xbd, 0x51, 0x25, 0x35, 0xb6, 0x6f, 0xe0, 0x9d });

HoloCameraStreamer::HoloCameraStreamer()
{
}


HoloCameraStreamer::~HoloCameraStreamer()
{
}

winrt::Windows::Foundation::IAsyncAction HoloCameraStreamer::StartStreamingAsync(
	winrt::Windows::Foundation::TypedEventHandler<winrt::Windows::Media::Capture::Frames::MediaFrameReader, winrt::Windows::Media::Capture::Frames::MediaFrameArrivedEventArgs> handler)
{
	//winrt::apartment_context ui_thread;

	//co_await winrt::resume_background();

	APTTYPE type;
	APTTYPEQUALIFIER qualifier;
	HRESULT const result = CoGetApartmentType(&type, &qualifier);


	
	auto media_capture = winrt::Windows::Media::Capture::MediaCapture();
	auto settings = winrt::Windows::Media::Capture::MediaCaptureInitializationSettings();
	settings.SharingMode(winrt::Windows::Media::Capture::MediaCaptureSharingMode::SharedReadOnly);
	settings.StreamingCaptureMode(winrt::Windows::Media::Capture::StreamingCaptureMode::Video);
	settings.MemoryPreference(winrt::Windows::Media::Capture::MediaCaptureMemoryPreference::Cpu);
	settings.StreamingCaptureMode(winrt::Windows::Media::Capture::StreamingCaptureMode::Video);


	auto media_sources{ co_await MediaFrameSourceGroup::FindAllAsync() };

	for (auto const& source_group : media_sources)
	{
		winrt::hstring name = source_group.DisplayName();
		if (name == L"Sensor Streaming")
		{
			// Found the group
			settings.SourceGroup(source_group);
			break;
		}
	}

	// (!settings.Source
	/*{
		throw std::runtime_error("Didn't find the Sensor Streaming source group");
	}

*/


	//co_await ui_thread;

	auto async_op = media_capture.InitializeAsync(settings);

	//co_await winrt::resume_background();
	try
	{
		co_await async_op;
	}
	catch (const winrt::hresult_error& e)
	{
		winrt::hstring msg = e.message();
		msg;
	}

	///co_await winrt::resume_background();

	// Iterate through discovered sources and find one of the cameras

	for (auto const& kvp : media_capture.FrameSources())
	{
		auto source = kvp.Value();
		auto user_data = source.Info().Properties().Lookup(c_MF_MT_USER_DATA);
        auto source_name = user_data.as<Windows::Foundation::IReferenceArray<std::uint8_t>>();
		winrt::com_array<std::uint8_t> arr;
		source_name.GetUInt8Array(arr);
		winrt::hstring source_name_str{ reinterpret_cast<wchar_t*>(arr.data()) };

		if (source_name_str == L"Visible Light Right-Front")
		{
			// Found it.
			_source = source;
			break;
		}
	}

	if (!_source)
	{
		throw std::runtime_error("Didn't find Visible Light Right-Front");
	}

	auto source_kind = _source.Info().SourceKind();

	if (source_kind != winrt::Windows::Media::Capture::Frames::MediaFrameSourceKind::Color)
	{
		throw std::runtime_error("Camera isn't image?");
	}

	winrt::Windows::Media::Capture::Frames::MediaFrameFormat _format = nullptr;
	winrt::hstring requested_subtype;
	//auto L8_encoding = Windows::
	for (auto const& format : _source.SupportedFormats())
	{
		auto format_subtype = format.Subtype();

		//if(format_subtype == Windows::Media::MediaProperties::MediaEncodingSubtypes::L8())
		//if (CompareStringOrdinal(format.Subtype().data(),
		//	-1,
		//	Windows::Media::MediaProperties::MediaEncodingSubtypes::L8->Data(),
		//	-1,
		//	TRUE) == CSTR_EQUAL)
		//{
			_format = format;
			requested_subtype = Windows::Media::MediaProperties::MediaEncodingSubtypes::Bgra8();
			break;
		//}
	}

	if (!_format)
	{
		throw std::runtime_error("Couldn't find L8 data");
	}

	_source.SetFormatAsync(_format).get();

	auto frame_reader = media_capture.CreateFrameReaderAsync(_source, requested_subtype).get();
	frame_reader.FrameArrived(handler);

	frame_reader.StartAsync();

	co_return;
}

