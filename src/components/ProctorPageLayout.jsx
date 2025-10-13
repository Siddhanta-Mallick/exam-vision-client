import Navbar from "./Navbar";
import ProctorMainWindow from "./ProctorMainWindow";

export default function ProctorPageLayout() {
    return (
        <div className="flex flex-col h-screen w-screen">
            <div><Navbar /></div>
            <div className="flex-1"><ProctorMainWindow /></div>
        </div>
    )
}
