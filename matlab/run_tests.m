function results = run_tests()
%RUN_TESTS  Run the unit tests in matlab/tests and print a summary table.
root = startup_paths();
if ~usejava('desktop'), set(0, 'DefaultFigureVisible', 'off'); end
results = runtests(fullfile(root, 'tests'));
disp(table(results));
end
