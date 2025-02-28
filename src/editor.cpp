

// System includes
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_win32.h"

// Local includes
#include "editor.hpp"

void Editor::helpMarker( const char* desc ) {
    ImGui::TextDisabled( "(?)" );
    if ( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) ) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos( ImGui::GetFontSize() * 35.0f );
        ImGui::TextUnformatted( desc );
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

Editor::Editor() {}

bool Editor::initialize( void* Window ) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // Setting up ImGui flags
    ImGuiIO& Io = ImGui::GetIO();
    Io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    Io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    Io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    // Setting style for ImGui
    ImGui::StyleColorsDark();
    if ( ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable ) {
        ImGuiStyle& Style = ImGui::GetStyle();
        Style.WindowRounding = 4.f;
        Style.ChildRounding = 4.f;
        Style.FrameRounding = 4.f;
        Style.GrabRounding = 4.f;
        Style.Colors[ImGuiCol_WindowBg].w = 1.f;
    }

    // Setting up ImGui
    ImGui_ImplWin32_InitForOpenGL( Window );
    ImGui_ImplOpenGL3_Init( "#version 330" );

    // Engine::Instance().AddUpdateCallback( std::bind( &Editor::Update, this )
    // ); Graphics::Instance().AddRenderCallback(
    //     std::bind( &Editor::Render, this ) );

    return true;
}

void Editor::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void Editor::update() {
    // ImGui update functions
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    // ImGui::ShowDemoWindow();

    // Display the different windows
    displayDockspace();

    for ( auto& func : DisplayMenuCallbacks ) {
        func();
    }
}

void Editor::render() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

    if ( ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable ) {
        // GLFWwindow* backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        // glfwMakeContextCurrent( backup_current_context );
    }
}

void Editor::displayDockspace() {
    // Setting up viewport
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos( viewport->Pos );
    ImGui::SetNextWindowSize( viewport->Size );
    ImGui::SetNextWindowViewport( viewport->ID );
    ImGui::SetNextWindowBgAlpha( 0.0f );

    // Setting up window flags
    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                    ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |=
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    // Setting up window style
    ImGui::PushStyleVar( ImGuiStyleVar_WindowRounding, 0.0f );
    ImGui::PushStyleVar( ImGuiStyleVar_WindowBorderSize, 0.0f );
    ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImVec2( 0.0f, 0.0f ) );

    // Making the window
    ImGui::SetNextWindowBgAlpha( 0.0f );
    ImGui::Begin( "Editor Window", &is_open, window_flags );
    ImGui::PopStyleVar( 3 );

    // Setting up window settings
    ImGuiID dockspace_id = ImGui::GetID( "Editor" );
    ImGuiDockNodeFlags dockspace_flags =
        ImGuiDockNodeFlags_PassthruCentralNode |
        ImGuiDockNodeFlags_NoDockingInCentralNode;
    ImGui::DockSpace( dockspace_id, ImVec2( 0.0f, 0.0f ), dockspace_flags );
    displayMenuBar();
    ImGui::End();
}

void Editor::displayMenuBar() {
    if ( ImGui::BeginMenuBar() ) {
        // if ( ImGui::BeginMenu( "File##1" ) ) {
        // if ( ImGui::MenuItem( "Save##1" ) ) {
        // Engine::Write();
        // }
        // if ( ImGui::MenuItem( "Save As..##1" ) ) {
        // ImGuiFileDialog::Instance()->OpenDialog( "ChooseFileDlgKey##7",
        // "Choose File", ".json", std::string( getenv( "USERPROFILE" ) ) +
        // "/Documents/pEngine/json/preset" );
        // }

        //     ImGui::EndMenu();
        // }

        ImGui::EndMenuBar();
    }
}

Editor& Editor::instance() {
    static Editor editorInstance;
    return editorInstance;
}
