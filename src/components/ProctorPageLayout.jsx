import Navbar from "./Navbar";
import ProctorMainWindow from "./ProctorMainWindow";

export default function ProctorPageLayout() {
    return (
        <div className="flex flex-col
        h-screen w-screen
        bg-[#0F0F0F] text-white">
            <div><Navbar /></div>
            <div className="flex-1 h-full"><ProctorMainWindow /></div>
        </div>
    )
}
